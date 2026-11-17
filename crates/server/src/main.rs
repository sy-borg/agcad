//! `agcad` — the headless server.
//!
//! Speaks line-delimited JSON-RPC 2.0 on **stdio** (default) or **TCP**
//! (`agcad --tcp 7860`). Each TCP connection gets its own engine; stdio
//! serves a single engine. Banner and diagnostics go to stderr, so stdout
//! stays a clean protocol channel — pipe-friendly for agents.
//!
//! Every muting command is answered with the full result (including fresh
//! mesh data), so a client never needs a second round trip to know what it
//! just created.

use std::io::{BufRead, BufReader, Write};
use std::net::{TcpListener, TcpStream};
use std::process::ExitCode;

use agcad_core::Engine;
use agcad_protocol::{Command, RpcError, RpcRequest, RpcResponse};
use serde_json::{Map, Value};

fn main() -> ExitCode {
    let mut tcp_port: Option<u16> = None;
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--tcp" => match args.next().and_then(|v| v.parse::<u16>().ok()) {
                Some(port) => tcp_port = Some(port),
                None => {
                    eprintln!("usage: agcad [--tcp PORT]");
                    return ExitCode::from(2);
                },
            },
            "--help" | "-h" => {
                eprintln!("usage: agcad [--tcp PORT]");
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

fn serve_stdio() -> ExitCode {
    eprintln!("agcad v0.1.0 | protocol: stdio | docs in-process");
    let stdin = std::io::stdin();
    let mut out = std::io::stdout().lock();

    let mut engine = Engine::new();
    for line in BufReader::new(stdin).lines() {
        match line {
            Ok(line) => {
                if line.trim().is_empty() {
                    continue;
                }
                if let Some(response) = handle_line(&mut engine, &line) {
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
    eprintln!("agcad v0.1.0 | protocol: tcp | listening on 127.0.0.1:{port}");

    for stream in listener.incoming() {
        match stream {
            Ok(stream) => {
                // One engine per connection: sessions are isolated, and a
                // crashed client can never poison another session.
                std::thread::spawn(move || {
                    if let Err(err) = handle_connection(stream) {
                        eprintln!("connection ended: {err}");
                    }
                });
            },
            Err(err) => eprintln!("accept error: {err}"),
        }
    }
    ExitCode::SUCCESS
}

fn handle_connection(stream: TcpStream) -> Result<(), std::io::Error> {
    let mut writer = stream.try_clone()?;
    let mut engine = Engine::new();
    for line in BufReader::new(stream).lines() {
        let line = line?;
        if line.trim().is_empty() {
            continue;
        }
        if let Some(response) = handle_line(&mut engine, &line) {
            writeln!(writer, "{response}")?;
        }
    }
    Ok(())
}

/// Parse one request line, execute it, serialize the response. Returns
/// `None` only when no usable request id could be recovered (pure noise).
fn handle_line(engine: &mut Engine, line: &str) -> Option<String> {
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
            .ok();
        },
    };

    let response = match to_command(&req) {
        Ok(command) => match engine.execute(command) {
            Ok(result) => RpcResponse::ok(req.id, result),
            Err(err) => RpcResponse::err(req.id, RpcError::invalid_params(err.to_string())),
        },
        Err(err) => {
            // serde reports unknown methods as "unknown variant".
            let code = if err.to_string().contains("unknown variant") {
                RpcError::method_not_found(format!("unknown method: {}", req.method))
            } else {
                RpcError::invalid_params(err.to_string())
            };
            RpcResponse::err(req.id, code)
        },
    };

    serde_json::to_string(&response).ok()
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
