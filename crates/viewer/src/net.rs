//! One-shot scene fetch over the command bus. The viewer is a pure
//! client: no side channel to the engine, ever.

use std::io::{BufRead, BufReader, Write};
use std::net::TcpStream;

use wright_protocol::{CommandResult, DocId, EntitySnapshot, PickHit, Ray, RpcRequest, RpcResponse};

pub fn fetch_scene(host: &str, port: u16) -> Result<(DocId, Vec<EntitySnapshot>), String> {
    let mut conn = Connection::open(host, port)?;

    let docs = match conn.call("list_documents", "{}")? {
        CommandResult::Documents { docs } => docs,
        other => return Err(format!("unexpected result: {other:?}")),
    };
    let Some(first) = docs.first() else {
        return Err("server has no open documents (create one with the Python client first)".into());
    };
    let doc = first.doc;

    let scene = match conn.call("get_scene", &format!(r#"{{"doc":{doc}}}"#))? {
        CommandResult::Scene { entities, .. } => entities,
        other => return Err(format!("unexpected result: {other:?}")),
    };
    Ok((doc, scene))
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
        let mut line = String::new();
        self.reader.read_line(&mut line).map_err(|e| e.to_string())?;
        if line.is_empty() {
            return Err("server closed the connection".into());
        }
        let resp: RpcResponse = serde_json::from_str(&line).map_err(|e| e.to_string())?;
        resp.result.ok_or_else(|| {
            resp.error.map(|e| e.message).unwrap_or_else(|| "empty response".into())
        })
    }
}
