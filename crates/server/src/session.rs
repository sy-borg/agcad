//! Project-scoped session descriptors for `Wright.open()`.
//!
//! A TCP server publishes its endpoint under the OS temp directory, keyed by
//! the directory it was started from. Scripts run from that directory (or a
//! subdirectory) can then find a non-default port without an environment
//! variable. The key is a 64-bit FNV-1a hash of the normalized path, which is
//! cheap to reproduce in the stdlib-only Python client and behaves the same
//! on Windows (case-folded, backslash-separated paths) and Unix.
//!
//! Writes are best-effort: a read-only temp directory must not stop the
//! server. Stale descriptors are harmless — the client tries the endpoint,
//! falls back when the connect fails, and removes the file.

use std::path::PathBuf;

const FNV_OFFSET: u64 = 0xcbf2_9ce4_8422_2325;
const FNV_PRIME: u64 = 0x0000_0100_0000_01b3;

/// FNV-1a over UTF-8 bytes. Kept in lockstep with `_session_key` in
/// `python/wright/client.py`; the unit test pins the value.
fn key(path: &str) -> String {
    let mut hash = FNV_OFFSET;
    for byte in path.as_bytes() {
        hash ^= u64::from(*byte);
        hash = hash.wrapping_mul(FNV_PRIME);
    }
    format!("{hash:016x}")
}

/// Normalize the working directory the way the Python client does:
/// absolute, backslash-separated and case-folded on Windows.
fn normalized_dir(dir: &std::path::Path) -> String {
    let path = dir.to_string_lossy().into_owned();
    #[cfg(windows)]
    {
        return path.replace('/', "\\").to_lowercase();
    }
    #[cfg(not(windows))]
    {
        path
    }
}

fn session_path(dir: &std::path::Path) -> Option<PathBuf> {
    let root = std::env::var_os("WRIGHT_SESSION_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| std::env::temp_dir().join("wright"));
    let key = key(&normalized_dir(dir));
    Some(root.join(format!("session-{key}.json")))
}

/// Publish `port` for the current working directory. Returns the descriptor
/// path when the write succeeded.
pub fn publish(port: u16) -> Option<PathBuf> {
    let cwd = std::env::current_dir().ok()?;
    let path = session_path(&cwd)?;
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent).ok()?;
    }
    let payload = serde_json::json!({
        "host": "127.0.0.1",
        "port": port,
        "pid": std::process::id(),
        "cwd": cwd.to_string_lossy(),
    });
    let text = serde_json::to_string(&payload).ok()?;
    let tmp = path.with_extension(format!("tmp{}", std::process::id()));
    std::fs::write(&tmp, text.as_bytes()).ok()?;
    // `rename` replaces an existing file on Unix, but not on Windows.
    #[cfg(windows)]
    {
        let _ = std::fs::remove_file(&path);
    }
    if std::fs::rename(&tmp, &path).is_err() {
        let _ = std::fs::write(&path, text.as_bytes());
        let _ = std::fs::remove_file(&tmp);
    }
    Some(path)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn key_matches_the_python_client() {
        // Values computed by python/wright/client.py `_session_key`; changing
        // either implementation must change this test.
        assert_eq!(key("C:\\project"), "1b78181f91d7ce21");
        assert_eq!(key("/home/sa/src/wright-cad"), "ca9f27a9e426936c");
    }

    #[test]
    fn descriptor_is_keyed_by_the_project_directory() {
        let root = std::path::Path::new("/tmp/wright-test");
        let a = session_path(root).unwrap();
        let b = session_path(std::path::Path::new("/tmp/wright-test/other")).unwrap();
        assert_ne!(a, b);
        assert!(a.to_string_lossy().contains("session-"));
    }
}
