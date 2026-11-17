//! One-shot scene fetch over the command bus. The viewer is a pure
//! client: no side channel to the engine, ever.

use std::io::{BufRead, BufReader, Write};
use std::net::TcpStream;

use agcad_protocol::{CommandResult, DocId, EntitySnapshot, RpcRequest, RpcResponse};

pub fn fetch_scene(host: &str, port: u16) -> Result<(DocId, Vec<EntitySnapshot>), String> {
    let stream =
        TcpStream::connect((host, port)).map_err(|e| format!("cannot connect to {host}:{port}: {e}"))?;
    let mut writer = stream.try_clone().map_err(|e| e.to_string())?;
    let mut reader = BufReader::new(stream);

    let mut call = |id: u64, method: &str, params: &str| -> Result<CommandResult, String> {
        let req = RpcRequest {
            jsonrpc: "2.0".into(),
            id,
            method: method.into(),
            params: serde_json::from_str(params).unwrap(),
        };
        writeln!(writer, "{}", serde_json::to_string(&req).unwrap()).map_err(|e| e.to_string())?;
        let mut line = String::new();
        reader.read_line(&mut line).map_err(|e| e.to_string())?;
        if line.is_empty() {
            return Err("server closed the connection".into());
        }
        let resp: RpcResponse = serde_json::from_str(&line).map_err(|e| e.to_string())?;
        resp.result.ok_or_else(|| {
            resp.error.map(|e| e.message).unwrap_or_else(|| "empty response".into())
        })
    };

    let docs = match call(1, "list_documents", "{}")? {
        CommandResult::Documents { docs } => docs,
        other => return Err(format!("unexpected result: {other:?}")),
    };
    let Some(first) = docs.first() else {
        return Err("server has no open documents (create one with the Python client first)".into());
    };
    let doc = first.doc;

    let scene = match call(2, "get_scene", &format!(r#"{{"doc":{doc}}}"#))? {
        CommandResult::Scene { entities, .. } => entities,
        other => return Err(format!("unexpected result: {other:?}")),
    };
    Ok((doc, scene))
}
