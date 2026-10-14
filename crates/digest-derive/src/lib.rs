// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Not in C: state digest of the chunked decoding.

//! `#[derive(StateDigest)]`: implements `mpeghdec_fdk::digest::StateDigest` by feeding every field
//! (every variant and its fields for enums) to the digest, in declaration order.
//!
//! Field attributes:
//! - `#[digest(skip)]`: the field is not part of the digest. Every skipped field needs a comment
//!   explaining why it cannot influence the future output of the decoder (e.g. scratch memory that
//!   is always written before it is read, or a counter that is only reported).
//! - `#[digest(with = path)]`: the field is fed with `path(&field, digester)`.
//!
//! Item attribute:
//! - `#[digest(extra = path)]`: after the fields, `path(&self, digester)` is called (for state
//!   that is part of the digest only depending on other fields).
//!
//! The macro has no dependencies: it parses the item with a small hand-written parser that
//! supports what the decoder uses (structs with named, unnamed or no fields, enums, lifetime /
//! type / const generic parameters, where clauses).

use proc_macro::{Delimiter, Spacing, TokenStream, TokenTree};

#[proc_macro_derive(StateDigest, attributes(digest))]
pub fn derive_state_digest(input: TokenStream) -> TokenStream {
    match derive(input) {
        Ok(ts) => ts,
        Err(msg) => format!("compile_error!({msg:?});").parse().unwrap(),
    }
}

enum Attr {
    Include,
    Skip,
    With(String),
    Extra(String),
}

struct Field {
    /// Field name, or its index for tuple fields.
    name: String,
    attr: Attr,
}

enum Fields {
    Named(Vec<Field>),
    Tuple(Vec<Field>),
    Unit,
}

const PATH: &str = "::mpeghdec_fdk::digest";

fn is_punct(t: Option<&TokenTree>, c: char) -> bool {
    matches!(t, Some(TokenTree::Punct(p)) if p.as_char() == c)
}

fn is_ident(t: Option<&TokenTree>, s: &str) -> bool {
    matches!(t, Some(TokenTree::Ident(i)) if i.to_string() == s)
}

fn to_string(toks: &[TokenTree]) -> String {
    toks.iter().cloned().collect::<TokenStream>().to_string()
}

/// Splits `toks` at the commas outside of angle brackets (`->` does not close a bracket).
fn split_commas(toks: &[TokenTree]) -> Vec<Vec<TokenTree>> {
    let mut out = Vec::new();
    let mut cur = Vec::new();
    let mut depth = 0i32;
    let mut prev_dash = false;
    for t in toks {
        let mut dash = false;
        if let TokenTree::Punct(p) = t {
            match p.as_char() {
                '<' => depth += 1,
                '>' if !prev_dash => depth -= 1,
                ',' if depth == 0 => {
                    out.push(std::mem::take(&mut cur));
                    prev_dash = false;
                    continue;
                }
                '-' => dash = p.spacing() == Spacing::Joint,
                _ => {}
            }
        }
        prev_dash = dash;
        cur.push(t.clone());
    }
    if !cur.is_empty() {
        out.push(cur);
    }
    out
}

/// Skips the outer attributes at `toks[*i..]`, returning the `digest` attribute.
fn attrs(toks: &[TokenTree], i: &mut usize) -> Result<Attr, String> {
    let mut attr = Attr::Include;
    while is_punct(toks.get(*i), '#') {
        let Some(TokenTree::Group(g)) = toks.get(*i + 1) else {
            return Err("StateDigest: malformed attribute".into());
        };
        let inner: Vec<TokenTree> = g.stream().into_iter().collect();
        if is_ident(inner.first(), "digest") {
            let Some(TokenTree::Group(args)) = inner.get(1) else {
                return Err("StateDigest: expected #[digest(...)]".into());
            };
            let args: Vec<TokenTree> = args.stream().into_iter().collect();
            if is_ident(args.first(), "skip") && args.len() == 1 {
                attr = Attr::Skip;
            } else if is_ident(args.first(), "with") && is_punct(args.get(1), '=') {
                attr = Attr::With(to_string(&args[2..]));
            } else if is_ident(args.first(), "extra") && is_punct(args.get(1), '=') {
                attr = Attr::Extra(to_string(&args[2..]));
            } else {
                return Err("StateDigest: expected #[digest(skip)], #[digest(with = path)] or \
                     #[digest(extra = path)]"
                    .into());
            }
        }
        *i += 2;
    }
    Ok(attr)
}

/// Skips a visibility at `toks[*i..]`.
fn visibility(toks: &[TokenTree], i: &mut usize) {
    if is_ident(toks.get(*i), "pub") {
        *i += 1;
        if let Some(TokenTree::Group(g)) = toks.get(*i) {
            if g.delimiter() == Delimiter::Parenthesis {
                *i += 1;
            }
        }
    }
}

fn parse_fields(group: &proc_macro::Group) -> Result<Fields, String> {
    let toks: Vec<TokenTree> = group.stream().into_iter().collect();
    let named = group.delimiter() == Delimiter::Brace;
    let mut fields = Vec::new();
    for (k, seg) in split_commas(&toks).into_iter().enumerate() {
        let mut i = 0;
        let attr = attrs(&seg, &mut i)?;
        visibility(&seg, &mut i);
        let name = if named {
            match seg.get(i) {
                Some(TokenTree::Ident(id)) => id.to_string(),
                _ => return Err("StateDigest: expected a field name".into()),
            }
        } else {
            k.to_string()
        };
        fields.push(Field { name, attr });
    }
    Ok(if named { Fields::Named(fields) } else { Fields::Tuple(fields) })
}

/// The destructuring pattern and the statements feeding the fields.
fn fields_code(path: &str, fields: &Fields) -> (String, String) {
    let var = |f: &Field| {
        if f.name.chars().next().is_some_and(|c| c.is_ascii_digit()) {
            format!("f{}", f.name)
        } else {
            f.name.clone()
        }
    };
    let mut body = String::new();
    let mut pat = Vec::new();
    let list = match fields {
        Fields::Unit => return (path.to_string(), String::new()),
        Fields::Named(l) | Fields::Tuple(l) => l,
    };
    for f in list {
        let v = var(f);
        let named = matches!(fields, Fields::Named(_));
        match &f.attr {
            Attr::Skip => pat.push(if named { format!("{}: _", f.name) } else { "_".into() }),
            _ => pat.push(if named { format!("{}: {v}", f.name) } else { v.clone() }),
        }
        match &f.attr {
            Attr::Skip | Attr::Extra(_) => {}
            Attr::Include => body += &format!("__d.field({:?}, {v});\n", f.name),
            Attr::With(p) => {
                body += &format!("__d.field_with({:?}, |__d| {p}({v}, __d));\n", f.name)
            }
        }
    }
    let pat = match fields {
        Fields::Named(_) => format!("{path} {{ {} }}", pat.join(", ")),
        _ => format!("{path}({})", pat.join(", ")),
    };
    (pat, body)
}

fn derive(input: TokenStream) -> Result<TokenStream, String> {
    let toks: Vec<TokenTree> = input.into_iter().collect();
    let mut i = 0;
    let extra = match attrs(&toks, &mut i)? {
        Attr::Extra(p) => Some(p),
        _ => None,
    };
    visibility(&toks, &mut i);
    let kind = match toks.get(i) {
        Some(TokenTree::Ident(id)) => id.to_string(),
        _ => return Err("StateDigest: expected struct or enum".into()),
    };
    i += 1;
    let name = match toks.get(i) {
        Some(TokenTree::Ident(id)) => id.to_string(),
        _ => return Err("StateDigest: expected a type name".into()),
    };
    i += 1;

    /* generic parameters */
    let mut impl_params = Vec::new();
    let mut ty_params = Vec::new();
    let mut bounds = Vec::new();
    if is_punct(toks.get(i), '<') {
        let start = i + 1;
        let mut depth = 1;
        let mut prev_dash = false;
        i += 1;
        while depth > 0 {
            match toks.get(i) {
                Some(TokenTree::Punct(p)) => {
                    match p.as_char() {
                        '<' => depth += 1,
                        '>' if !prev_dash => depth -= 1,
                        _ => {}
                    }
                    prev_dash = p.as_char() == '-';
                }
                Some(_) => prev_dash = false,
                None => return Err("StateDigest: unterminated generics".into()),
            }
            i += 1;
        }
        for p in split_commas(&toks[start..i - 1]) {
            /* drop a default (`T = X`) */
            let end = p.iter().position(|t| is_punct(Some(t), '=')).unwrap_or(p.len());
            let p = &p[..end];
            impl_params.push(to_string(p));
            if is_punct(p.first(), '\'') {
                ty_params.push(to_string(&p[..2]));
            } else if is_ident(p.first(), "const") {
                ty_params.push(to_string(&p[1..2]));
            } else {
                let n = to_string(&p[..1]);
                bounds.push(format!("{n}: {PATH}::StateDigest"));
                ty_params.push(n);
            }
        }
    }

    /* where clause and body */
    let mut where_toks = Vec::new();
    let mut body = None;
    while let Some(t) = toks.get(i) {
        match t {
            TokenTree::Group(g)
                if matches!(g.delimiter(), Delimiter::Brace | Delimiter::Parenthesis) =>
            {
                if body.is_none() {
                    body = Some(g.clone());
                }
            }
            TokenTree::Punct(p) if p.as_char() == ';' => {}
            _ if is_ident(Some(t), "where") => {}
            _ => where_toks.push(t.clone()),
        }
        i += 1;
    }
    let mut where_clause = to_string(&where_toks).trim().trim_end_matches(',').to_string();
    for b in bounds {
        if !where_clause.is_empty() {
            where_clause += ", ";
        }
        where_clause += &b;
    }

    let code = match kind.as_str() {
        "struct" => {
            let fields = match &body {
                Some(g) => parse_fields(g)?,
                None => Fields::Unit,
            };
            let (pat, stmts) = fields_code("Self", &fields);
            let extra = extra.as_ref().map(|p| format!("{p}(self, __d);")).unwrap_or_default();
            format!("let {pat} = self;\n{stmts}\n{extra}")
        }
        "enum" => {
            let Some(g) = &body else { return Err("StateDigest: enum without body".into()) };
            let vtoks: Vec<TokenTree> = g.stream().into_iter().collect();
            let mut arms = String::new();
            for (k, seg) in split_commas(&vtoks).into_iter().enumerate() {
                let mut j = 0;
                attrs(&seg, &mut j)?;
                let vname = match seg.get(j) {
                    Some(TokenTree::Ident(id)) => id.to_string(),
                    _ => return Err("StateDigest: expected a variant name".into()),
                };
                let fields = match seg.get(j + 1) {
                    Some(TokenTree::Group(fg))
                        if matches!(fg.delimiter(), Delimiter::Brace | Delimiter::Parenthesis) =>
                    {
                        parse_fields(fg)?
                    }
                    _ => Fields::Unit,
                };
                let (pat, stmts) = fields_code(&format!("Self::{vname}"), &fields);
                arms += &format!("{pat} => {{ __d.u32({k}); {stmts} }}\n");
            }
            if arms.is_empty() {
                "match *self {}".to_string()
            } else {
                format!("match self {{ {arms} }}")
            }
        }
        _ => return Err(format!("StateDigest: cannot derive for {kind}")),
    };

    let impl_generics = if impl_params.is_empty() {
        String::new()
    } else {
        format!("<{}>", impl_params.join(", "))
    };
    let ty_generics =
        if ty_params.is_empty() { String::new() } else { format!("<{}>", ty_params.join(", ")) };
    let where_clause =
        if where_clause.is_empty() { String::new() } else { format!("where {where_clause}") };
    let out = format!(
        "#[automatically_derived]
        impl{impl_generics} {PATH}::StateDigest for {name}{ty_generics} {where_clause} {{
            #[allow(unused_variables, clippy::all)]
            fn digest(&self, __d: &mut {PATH}::Digester) {{
                {code}
            }}
        }}"
    );
    out.parse().map_err(|e| format!("StateDigest: generated code does not parse: {e:?}"))
}
