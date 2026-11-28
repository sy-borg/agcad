//! One-shot scene fetch over the command bus. The viewer is a pure
//! client: no side channel to the engine, ever.

use std::io::{BufRead, BufReader, Write};
use std::net::TcpStream;

use wright_protocol::{
    CommandResult, DocId, DocInfo, EntitySnapshot, PickHit, Ray, RpcRequest, RpcResponse,
};

/// What the viewer needs for one refresh: the available documents, the
/// resolved selection, and that document's entities. An empty document list
/// is a valid snapshot (the viewer waits for one to be created), not an error.
pub struct FetchedScene {
    pub docs: Vec<DocInfo>,
    pub doc: Option<DocId>,
    pub entities: Vec<EntitySnapshot>,
}

/// Resolve a viewer's preferred document against the live document list.
/// An explicit `want` wins while it still exists; otherwise the first open
/// document is the fallback (what an unselected viewer follows).
pub fn select_document(docs: &[DocInfo], want: Option<DocId>) -> Option<DocId> {
    want.filter(|id| docs.iter().any(|d| d.doc == *id))
        .or_else(|| docs.first().map(|d| d.doc))
}

pub fn fetch_scene(host: &str, port: u16, want: Option<DocId>) -> Result<FetchedScene, String> {
    let mut conn = Connection::open(host, port)?;

    let docs = match conn.call("list_documents", "{}")? {
        CommandResult::Documents { docs } => docs,
        other => return Err(format!("unexpected result: {other:?}")),
    };
    let doc = select_document(&docs, want);

    let entities = match doc {
        Some(doc) => match conn.call("get_scene", &format!(r#"{{"doc":{doc}}}"#))? {
            CommandResult::Scene { entities, .. } => entities,
            other => return Err(format!("unexpected result: {other:?}")),
        },
        None => Vec::new(),
    };
    Ok(FetchedScene { docs, doc, entities })
}

/// Ask the engine what is under a world-space ray. A miss is `Ok(None)`;
/// transport and protocol failures are errors the viewer can report.
pub fn pick(
    host: &str,
    port: u16,
    doc: DocId,
    ray: Ray,
    tolerance: f64,
) -> Result<Option<PickHit>, String> {
    let mut conn = Connection::open(host, port)?;
    let params = serde_json::json!({
        "doc": doc,
        "ray": { "origin": ray.origin, "direction": ray.direction },
        "tolerance": tolerance,
    })
    .to_string();
    match conn.call("pick", &params)? {
        CommandResult::Picked { hit, .. } => Ok(hit),
        other => Err(format!("unexpected result: {other:?}")),
    }
}

/// One persistent connection for a single viewer operation.
struct Connection {
    writer: TcpStream,
    reader: BufReader<TcpStream>,
    next_id: u64,
}

impl Connection {
    fn open(host: &str, port: u16) -> Result<Self, String> {
        let stream =
            TcpStream::connect((host, port)).map_err(|e| format!("cannot connect to {host}:{port}: {e}"))?;
        let writer = stream.try_clone().map_err(|e| e.to_string())?;
        Ok(Self { writer, reader: BufReader::new(stream), next_id: 1 })
    }

    fn call(&mut self, method: &str, params: &str) -> Result<CommandResult, String> {
        let id = self.next_id;
        self.next_id += 1;
        let req = RpcRequest {
            jsonrpc: "2.0".into(),
            id,
            method: method.into(),
            params: serde_json::from_str(params).unwrap(),
        };
        writeln!(self.writer, "{}", serde_json::to_string(&req).unwrap()).map_err(|e| e.to_string())?;
        loop {
            let mut line = String::new();
            self.reader.read_line(&mut line).map_err(|e| e.to_string())?;
            if line.is_empty() {
                return Err("server closed the connection".into());
            }
            let value: serde_json::Value =
                serde_json::from_str(&line).map_err(|e| e.to_string())?;
            // The server fans `scene_changed` out to every open connection,
            // including this short-lived one. A mutation made by another
            // client can therefore put a notification (no `id`) on the wire
            // before our response; skip it instead of failing to parse it as
            // an `RpcResponse`.
            if value.get("id").is_none() {
                continue;
            }
            let resp: RpcResponse = serde_json::from_value(value).map_err(|e| e.to_string())?;
            if resp.id != id {
                continue;
            }
            return resp.result.ok_or_else(|| {
                resp.error.map(|e| e.message).unwrap_or_else(|| "empty response".into())
            });
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::net::TcpListener;

    /// A mutation by another client is broadcast to every open connection,
    /// including the viewer's one-shot scene fetch. The notification can land
    /// before (or between) our responses; it must be skipped, not reported as
    /// `missing field id`.
    #[test]
    fn notifications_do_not_break_a_scene_fetch() {
        let listener = TcpListener::bind("127.0.0.1:0").expect("bind fake backend");
        let port = listener.local_addr().expect("local addr").port();

        std::thread::spawn(move || {
            let (stream, _) = listener.accept().expect("accept");
            let mut writer = stream.try_clone().expect("clone stream");
            let mut reader = BufReader::new(stream);
            for id in 1..=2 {
                let mut request = String::new();
                reader.read_line(&mut request).expect("read request");
                assert!(request.contains("\"list_documents\""), "{request}");
                // Another client mutates while this connection waits.
                writeln!(
                    writer,
                    r#"{{"jsonrpc":"2.0","method":"scene_changed","params":{{"doc":{id}}}}}"#
                )
                .expect("write notification");
                writeln!(
                    writer,
                    r#"{{"jsonrpc":"2.0","id":{id},"result":{{"documents":{{"docs":[]}}}}}}"#
                )
                .expect("write response");
            }
        });

        let mut conn = Connection::open("127.0.0.1", port).expect("connect");
        for _ in 0..2 {
            match conn.call("list_documents", "{}").expect("response despite notification") {
                CommandResult::Documents { docs } => assert!(docs.is_empty()),
                other => panic!("unexpected result: {other:?}"),
            }
        }
    }
}
