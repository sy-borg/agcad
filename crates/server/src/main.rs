//! `wright` — the headless server.
//!
//! Speaks line-delimited JSON-RPC 2.0 on **stdio** (default) or **TCP**
//! (`wright --tcp 7860`). Banner and diagnostics go to stderr, so stdout
//! stays a clean protocol channel — pipe-friendly for agents.
//!
//! Engine sharing:
//! - stdio: one engine for the single client (isolated, private).
//! - TCP: ONE shared engine — every connection sees the same documents,
//!   so an agent drawing over MCP is instantly visible to a viewer.
//!   After every mutating command the server broadcasts a `scene_changed`
//!   JSON-RPC notification (no `id`) to all connections; viewers listen
//!   and refresh themselves.

use std::io::{BufRead, BufReader, Write};
use std::net::{TcpListener, TcpStream};
use std::process::ExitCode;
use std::sync::{Arc, Mutex, mpsc};

use wright_core::Engine;
use wright_protocol::{Command, CommandResult, DocId, RpcError, RpcRequest, RpcResponse};
use serde_json::{Map, Value, json};

fn main() -> ExitCode {
    let mut tcp_port: Option<u16> = None;
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--tcp" => match args.next().and_then(|v| v.parse::<u16>().ok()) {
                Some(port) => tcp_port = Some(port),
                None => {
                    eprintln!("usage: wright [--tcp PORT]");
                    return ExitCode::from(2);
                },
            },
            "--help" | "-h" => {
                eprintln!("usage: wright [--tcp PORT]");
                return ExitCode::SUCCESS;
            },
            other => {
                eprintln!("unknown argument: {other}");
                return ExitCode::from(2);
            },
        }
    }

    match tcp_port {
        Some(port) => serve_tcp(port),
        None => serve_stdio(),
    }
}

// --- notification fan-out ------------------------------------------------------

/// Fan-out for `scene_changed` notifications. Sinks are `Arc`'d so dead
/// connections can be removed by identity.
struct Broadcaster {
    sinks: Mutex<Vec<Arc<mpsc::Sender<String>>>>,
}

impl Broadcaster {
    fn new() -> Self {
        Self { sinks: Mutex::new(Vec::new()) }
    }

    fn add(&self, sink: Arc<mpsc::Sender<String>>) {
        self.sinks.lock().unwrap().push(sink);
    }

    fn remove(&self, sink: &Arc<mpsc::Sender<String>>) {
        self.sinks.lock().unwrap().retain(|s| !Arc::ptr_eq(s, sink));
    }

    fn send_except(&self, except: &Arc<mpsc::Sender<String>>, line: &str) {
        for sink in self.sinks.lock().unwrap().iter() {
            if !Arc::ptr_eq(sink, except) {
                let _ = sink.send(line.to_string());
            }
        }
    }

    #[allow(dead_code)] // kept for future all-client broadcasts
    fn send(&self, line: &str) {
        for sink in self.sinks.lock().unwrap().iter() {
            let _ = sink.send(line.to_string());
        }
    }
}

fn is_mutating(command: &Command) -> bool {
    matches!(
        command,
        Command::AddBox { .. }
            | Command::AddSphere { .. }
            | Command::AddCylinder { .. }
            | Command::ImportMesh { .. }
            | Command::ImportDxf { .. }
            | Command::ImportSvg { .. }
            | Command::AddMesh { .. }
            | Command::Transform { .. }
            | Command::SetColor { .. }
            | Command::Delete { .. }
            | Command::Fillet { .. }
            | Command::Chamfer { .. }
            | Command::Boolean { .. }
            | Command::AddSketch { .. }
            | Command::AddSketchCurves { .. }
            | Command::Section { .. }
            | Command::Project { .. }
            | Command::RepairMesh { .. }
            | Command::AddSketchLine { .. }
            | Command::AddSketchCircle { .. }
            | Command::AddSketchArc { .. }
            | Command::AddSketchSpline { .. }
            | Command::TrimSketch { .. }
            | Command::ExtendSketch { .. }
            | Command::OffsetSketch { .. }
            | Command::Constrain { .. }
            | Command::RemoveConstraint { .. }
            | Command::MoveSketchPoint { .. }
            | Command::DeleteSketchElement { .. }
            | Command::Extrude { .. }
            | Command::Revolve { .. }
            | Command::LinearPattern { .. }
            | Command::CircularPattern { .. }
            | Command::Mirror { .. }
            | Command::Shell { .. }
            | Command::Hole { .. }
            | Command::Sweep { .. }
            | Command::Loft { .. }
            | Command::Thread { .. }
            | Command::ImportPart { .. }
            | Command::AddPart { .. }
            | Command::AddInstance { .. }
            | Command::AddConnector { .. }
            | Command::MoveInstance { .. }
            | Command::Mate { .. }
            | Command::SetParameter { .. }
            | Command::RemoveParameter { .. }
            | Command::CreateDocument { .. }
            | Command::OpenDocument { .. }
            | Command::CloseDocument { .. }
            | Command::Undo { .. }
            | Command::Redo { .. }
            | Command::PatchProgram { .. }
            | Command::Suppress { .. }
            | Command::Batch { .. }
    )
}

fn doc_of(command: &Command) -> Option<DocId> {
    match command {
        Command::AddBox { doc, .. }
        | Command::AddSphere { doc, .. }
        | Command::AddCylinder { doc, .. }
        | Command::ImportMesh { doc, .. }
        | Command::ImportDxf { doc, .. }
        | Command::ImportSvg { doc, .. }
        | Command::AddMesh { doc, .. }
        | Command::Transform { doc, .. }
        | Command::SetColor { doc, .. }
        | Command::SetMaterial { doc, .. }
        | Command::Delete { doc, .. }
        | Command::Fillet { doc, .. }
        | Command::Chamfer { doc, .. }
        | Command::Boolean { doc, .. }
        | Command::AddSketch { doc, .. }
        | Command::AddSketchCurves { doc, .. }
        | Command::Section { doc, .. }
        | Command::Project { doc, .. }
        | Command::RepairMesh { doc, .. }
        | Command::AddSketchLine { doc, .. }
        | Command::AddSketchCircle { doc, .. }
        | Command::AddSketchArc { doc, .. }
        | Command::AddSketchSpline { doc, .. }
        | Command::TrimSketch { doc, .. }
        | Command::ExtendSketch { doc, .. }
        | Command::OffsetSketch { doc, .. }
        | Command::Constrain { doc, .. }
        | Command::RemoveConstraint { doc, .. }
        | Command::MoveSketchPoint { doc, .. }
        | Command::DeleteSketchElement { doc, .. }
        | Command::Extrude { doc, .. }
        | Command::Revolve { doc, .. }
        | Command::LinearPattern { doc, .. }
        | Command::CircularPattern { doc, .. }
        | Command::Mirror { doc, .. }
        | Command::Shell { doc, .. }
        | Command::Hole { doc, .. }
        | Command::Sweep { doc, .. }
        | Command::Loft { doc, .. }
        | Command::Thread { doc, .. }
        | Command::ImportPart { doc, .. }
        | Command::AddPart { doc, .. }
        | Command::AddInstance { doc, .. }
        | Command::AddConnector { doc, .. }
        | Command::MoveInstance { doc, .. }
        | Command::Mate { doc, .. }
        | Command::BillOfMaterials { doc }
        | Command::CheckInterference { doc, .. }
        | Command::ExportDrawing { doc, .. }
        | Command::SetParameter { doc, .. }
        | Command::RemoveParameter { doc, .. }
        | Command::Undo { doc }
        | Command::Redo { doc }
        | Command::GetScene { doc }
        | Command::Pick { doc, .. }
        | Command::Faces { doc, .. }
        | Command::Edges { doc, .. }
        | Command::Primitives { doc, .. }
        | Command::Validate { doc, .. }
        | Command::SuggestOrientation { doc, .. }
        | Command::Arrange { doc, .. }
        | Command::Fit { doc, .. }
        | Command::Measure { doc, .. }
        | Command::MassProperties { doc, .. }
        | Command::Clearance { doc, .. }
        | Command::DiffScene { doc, .. }
        | Command::GetParameters { doc }
        | Command::ExportStl { doc, .. }
        | Command::Export3mf { doc, .. }
        | Command::ExportDxf { doc, .. }
        | Command::ExportSvg { doc, .. }
        | Command::ExportStep { doc, .. }
        | Command::SaveDocument { doc, .. }
        | Command::GetProgram { doc }
        | Command::RenderOffscreen { doc, .. }
        | Command::SetCalibration { doc, .. }
        | Command::CompareRender { doc, .. }
        | Command::PatchProgram { doc, .. }
        | Command::Suppress { doc, .. }
        | Command::CloseDocument { doc } => Some(*doc),
        Command::Batch { commands, .. } => commands.iter().find_map(doc_of),
        Command::CreateDocument { .. }
        | Command::OpenDocument { .. }
        | Command::ListDocuments
        | Command::ListMaterials
        | Command::Initialize => None,
    }
}

fn scene_changed(command: &Command, result: &CommandResult) -> String {
    // Create/open have no doc in their params; the result names the new
    // document, so viewers can react to it without re-listing blindly.
    let doc = doc_of(command).or_else(|| match result {
        CommandResult::DocumentCreated { doc }
        | CommandResult::DocumentOpened { doc, .. } => Some(*doc),
        _ => None,
    });
    let params = match doc {
        Some(doc) => json!({ "doc": doc }),
        None => json!({}),
    };
    json!({ "jsonrpc": "2.0", "method": "scene_changed", "params": params }).to_string()
}

// --- modes ---------------------------------------------------------------------

fn serve_stdio() -> ExitCode {
    eprintln!(
        "Wright CAD v{} | protocol: stdio | engine: private to this session",
        env!("CARGO_PKG_VERSION")
    );
    let stdin = std::io::stdin();
    let mut out = std::io::stdout().lock();

    let engine = Mutex::new(Engine::new());
    for line in BufReader::new(stdin).lines() {
        match line {
            Ok(line) => {
                if line.trim().is_empty() {
                    continue;
                }
                if let Some((response, _notify)) = handle_line(&engine, &line, None) {
                    if writeln!(out, "{response}").is_err() {
                        // Client went away.
                        return ExitCode::SUCCESS;
                    }
                    let _ = out.flush();
                }
            },
            Err(err) => {
                eprintln!("stdio read error: {err}");
                return ExitCode::SUCCESS;
            },
        }
    }
    ExitCode::SUCCESS
}

fn serve_tcp(port: u16) -> ExitCode {
    let listener = match TcpListener::bind(("127.0.0.1", port)) {
        Ok(l) => l,
        Err(err) => {
            eprintln!("cannot bind 127.0.0.1:{port}: {err}");
            return ExitCode::FAILURE;
        },
    };
    eprintln!(
        "Wright CAD v{} | protocol: tcp | listening on 127.0.0.1:{port}",
        env!("CARGO_PKG_VERSION")
    );
    eprintln!("docs: one shared document space for every connection; mutations broadcast scene_changed");

    let engine = Arc::new(Mutex::new(Engine::new()));
    let bus = Arc::new(Broadcaster::new());

    for stream in listener.incoming() {
        match stream {
            Ok(stream) => {
                let engine = engine.clone();
                let bus = bus.clone();
                std::thread::spawn(move || {
                    if let Err(err) = handle_connection(stream, engine, bus) {
                        eprintln!("connection ended: {err}");
                    }
                });
            },
            Err(err) => eprintln!("accept error: {err}"),
        }
    }
    ExitCode::SUCCESS
}

type SharedEngine = Arc<Mutex<Engine>>;

fn handle_connection(
    stream: TcpStream,
    engine: SharedEngine,
    bus: Arc<Broadcaster>,
) -> Result<(), std::io::Error> {
    let writer = stream.try_clone()?;
    let (tx, rx) = mpsc::channel::<String>();
    let tx = Arc::new(tx);
    bus.add(tx.clone());

    // Writer thread: serializes responses + notifications onto the socket,
    // so the read loop never blocks on a slow client.
    std::thread::spawn(move || {
        let mut writer = writer;
        for msg in rx {
            if writeln!(writer, "{msg}").is_err() {
                break;
            }
        }
    });

    for line in BufReader::new(stream).lines() {
        let line = line?;
        if line.trim().is_empty() {
            continue;
        }
        if let Some((response, notify)) = handle_line(&engine, &line, Some(&bus)) {
            // The caller gets its response; other connections get the
            // scene_changed notification. The caller already knows it
            // changed the scene — echoing it back would pollute its stream.
            let _ = tx.send(response);
            if let Some(note) = notify {
                bus.send_except(&tx, &note);
            }
        }
    }
    bus.remove(&tx);
    Ok(())
}

// --- request handling ------------------------------------------------------------

/// Parse one request line, execute it, serialize the response. Returns
/// the response plus a `scene_changed` notification to fan out on success
/// of a mutating command (so callers can order response-before-notify).
fn handle_line(
    engine: &Mutex<Engine>,
    line: &str,
    _bus: Option<&Broadcaster>,
) -> Option<(String, Option<String>)> {
    let req: RpcRequest = match serde_json::from_str(line) {
        Ok(req) => req,
        Err(err) => {
            // Recover the id if we can, else answer with id 0 so the client
            // learns about the failure instead of hanging.
            let id = serde_json::from_str::<Value>(line)
                .ok()
                .and_then(|v| v.get("id").and_then(Value::as_u64))
                .unwrap_or(0);
            return serde_json::to_string(&RpcResponse::err(
                id,
                RpcError::parse(format!("unparseable request: {err}")),
            ))
            .ok()
            .map(|response| (response, None));
        },
    };

    let response = match to_command(&req) {
        Ok(command) => {
            // Create/open know their document only after they run, so keep a
            // copy of mutating commands and build the notification from the
            // result rather than from the request params.
            let notify_for = is_mutating(&command).then(|| command.clone());
            match engine.lock().unwrap().execute(command) {
                Ok(result) => {
                    let notify = notify_for
                        .as_ref()
                        .map(|command| scene_changed(command, &result));
                    (RpcResponse::ok(req.id, result), notify)
                },
                Err(err) => (RpcResponse::err(req.id, RpcError::invalid_params(err.to_string())), None),
            }
        },
        Err(err) => {
            // serde reports unknown methods as "unknown variant".
            let code = if err.to_string().contains("unknown variant") {
                RpcError::method_not_found(format!("unknown method: {}", req.method))
            } else {
                RpcError::invalid_params(err.to_string())
            };
            (RpcResponse::err(req.id, code), None)
        },
    };

    let (response, notify) = response;
    let response = serde_json::to_string(&response).ok()?;
    // `notify` is already a serialized JSON-RPC line.
    Some((response, notify))
}

/// JSON-RPC `method` + `params` -> [`Command`]. The method tag is merged
/// into the params object so the protocol enum deserializes directly.
fn to_command(req: &RpcRequest) -> Result<Command, serde_json::Error> {
    let mut params = match &req.params {
        Value::Object(map) => map.clone(),
        Value::Null => Map::new(),
        _ => {
            return Err(serde::de::Error::custom(format!(
                "params must be a JSON object (method: {})",
                req.method
            )))
        },
    };
    params.insert("method".into(), Value::String(req.method.clone()));
    serde_json::from_value(Value::Object(params))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn notified_doc(line: &str) -> Option<u64> {
        let value: Value = serde_json::from_str(line).expect("notification json");
        assert_eq!(value["method"], "scene_changed");
        value["params"].get("doc").and_then(Value::as_u64)
    }

    #[test]
    fn create_and_open_notifications_name_the_new_document() {
        let create = Command::CreateDocument { name: "bracket".into(), doc: None };
        let created = CommandResult::DocumentCreated { doc: 7 };
        assert_eq!(notified_doc(&scene_changed(&create, &created)), Some(7));

        let open = Command::OpenDocument { path: "/tmp/a.wcad".into(), name: None };
        let opened =
            CommandResult::DocumentOpened { doc: 8, name: "a".into(), commands: 1, params: 0, entities: 1 };
        assert_eq!(notified_doc(&scene_changed(&open, &opened)), Some(8));
    }
}
