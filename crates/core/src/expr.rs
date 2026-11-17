//! Tiny, safe arithmetic evaluator for parametric expressions.
//!
//! Grammar (no functions, no assignment, no side channels — deterministic
//! by construction):
//!
//! ```text
//! expr   := term  (('+' | '-') term)*
//! term   := unary (('*' | '/') unary)*
//! unary  := ('-' | '+') unary | primary
//! primary:= NUMBER | IDENT | '(' expr ')'
//! ```
//!
//! Identifiers resolve against the document's parameter table. Unknown
//! identifiers produce an error that *lists the available parameters* —
//! agent ergonomics: the fix is in the message.

use std::collections::BTreeMap;

use crate::Param;

pub fn eval(src: &str, params: &BTreeMap<String, Param>) -> Result<f64, String> {
    let tokens = tokenize(src)?;
    if tokens.is_empty() {
        return Err(format!("empty expression"));
    }
    let mut p = Parser { tokens: &tokens, pos: 0, params, src };
    let value = p.expr()?;
    if p.pos != p.tokens.len() {
        return Err(format!(
            "unexpected {:?} after expression in '{src}' (check operator placement)",
            p.tokens[p.pos]
        ));
    }
    Ok(value)
}

#[derive(Debug, Clone, PartialEq)]
enum Tok {
    Num(f64),
    Ident(String),
    Op(char), // + - * / ( )
}

fn tokenize(src: &str) -> Result<Vec<Tok>, String> {
    let mut out = Vec::new();
    let mut chars = src.chars().peekable();
    while let Some(&c) = chars.peek() {
        match c {
            ' ' | '\t' => {
                chars.next();
            },
            '+' | '-' | '*' | '/' | '(' | ')' => {
                out.push(Tok::Op(c));
                chars.next();
            },
            '0'..='9' | '.' => {
                let mut num = String::new();
                while let Some(&c) = chars.peek() {
                    if c.is_ascii_digit() || c == '.' {
                        num.push(c);
                        chars.next();
                    } else {
                        break;
                    }
                }
                let v: f64 = num
                    .parse()
                    .map_err(|_| format!("invalid number '{num}' in expression '{src}'"))?;
                out.push(Tok::Num(v));
            },
            c if c.is_alphabetic() || c == '_' => {
                let mut ident = String::new();
                while let Some(&c) = chars.peek() {
                    if c.is_alphanumeric() || c == '_' {
                        ident.push(c);
                        chars.next();
                    } else {
                        break;
                    }
                }
                out.push(Tok::Ident(ident));
            },
            other => {
                return Err(format!(
                    "unexpected character '{other}' in expression '{src}'"
                ))
            },
        }
    }
    Ok(out)
}

struct Parser<'a> {
    tokens: &'a [Tok],
    pos: usize,
    params: &'a BTreeMap<String, Param>,
    src: &'a str,
}

impl<'a> Parser<'a> {
    fn peek(&self) -> Option<&Tok> {
        self.tokens.get(self.pos)
    }

    fn bump(&mut self) -> Option<&Tok> {
        let t = self.tokens.get(self.pos);
        if t.is_some() {
            self.pos += 1;
        }
        t
    }

    fn expr(&mut self) -> Result<f64, String> {
        let mut v = self.term()?;
        while let Some(Tok::Op(op @ ('+' | '-'))) = self.peek() {
            let op = *op;
            self.pos += 1;
            let rhs = self.term()?;
            v = if op == '+' { v + rhs } else { v - rhs };
        }
        Ok(v)
    }

    fn term(&mut self) -> Result<f64, String> {
        let mut v = self.unary()?;
        while let Some(Tok::Op(op @ ('*' | '/'))) = self.peek() {
            let op = *op;
            self.pos += 1;
            let rhs = self.unary()?;
            if op == '/' && rhs == 0.0 {
                return Err(format!("division by zero in expression '{}'", self.src));
            }
            v = if op == '*' { v * rhs } else { v / rhs };
        }
        Ok(v)
    }

    fn unary(&mut self) -> Result<f64, String> {
        let tok = self.bump().cloned();
        match tok {
            Some(Tok::Op('-')) => Ok(-self.unary()?),
            Some(Tok::Op('+')) => self.unary(),
            _ => {
                self.pos -= 1;
                self.primary()
            },
        }
    }

    fn primary(&mut self) -> Result<f64, String> {
        let tok = self.bump().cloned();
        match tok {
            Some(Tok::Num(v)) => Ok(v),
            Some(Tok::Ident(name)) => match self.params.get(&name) {
                Some(p) => Ok(p.value),
                None => {
                    let mut available: Vec<String> = self.params.keys().cloned().collect();
                    available.sort();
                    let hint = if available.is_empty() {
                        "no parameters defined yet (hint: set_parameter first)".to_string()
                    } else {
                        format!("available parameters: {}", available.join(", "))
                    };
                    Err(format!(
                        "unknown identifier '{name}' in expression '{}'; {hint}",
                        self.src
                    ))
                },
            },
            Some(Tok::Op('(')) => {
                let v = self.expr()?;
                match self.bump().cloned() {
                    Some(Tok::Op(')')) => Ok(v),
                    other => Err(format!(
                        "missing closing parenthesis in '{}' (found {other:?})",
                        self.src
                    )),
                }
            },
            other => Err(format!("unexpected {other:?} in expression '{}'", self.src)),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn params(entries: &[(&str, f64)]) -> BTreeMap<String, Param> {
        entries
            .iter()
            .map(|(k, v)| (k.to_string(), Param { value: *v, description: None }))
            .collect()
    }

    #[test]
    fn arithmetic() {
        let p = params(&[("wall", 2.0), ("plate", 80.0)]);
        assert_eq!(eval("wall*2", &p).unwrap(), 4.0);
        assert_eq!(eval("plate/2 + wall", &p).unwrap(), 42.0);
        assert_eq!(eval("wall + wall*3", &p).unwrap(), 8.0, "precedence");
        assert_eq!(eval("(plate + 20) / 2", &p).unwrap(), 50.0);
        assert_eq!(eval("-wall + 5", &p).unwrap(), 3.0);
        assert_eq!(eval("wall", &p).unwrap(), 2.0, "bare identifier");
    }

    #[test]
    fn errors_are_helpful() {
        let p = params(&[("wall", 2.0)]);
        let err = eval("walll*2", &p).unwrap_err();
        assert!(err.contains("'walll'") && err.contains("available parameters: wall"), "{err}");

        let empty = params(&[]);
        let err = eval("wall*2", &empty).unwrap_err();
        assert!(err.contains("no parameters defined"), "{err}");

        assert!(eval("1 + ", &params(&[])).is_err());
        assert!(eval("1 / 0", &params(&[])).is_err());
        assert!(eval("import os", &params(&[])).is_err());
        assert!(eval("", &params(&[])).is_err());
    }

    #[test]
    fn rejects_injection() {
        let p = params(&[("a", 1.0)]);
        for bad in ["a;a", "a=2", "__import__", "a(b)", "0x1f"] {
            // Must either error or evaluate as pure arithmetic — never execute.
            assert!(eval(bad, &p).is_err() || eval(bad, &p).unwrap().is_finite(), "{bad}");
        }
    }
}
