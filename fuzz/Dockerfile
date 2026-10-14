# Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
# Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
# See LICENSE.txt for the complete license text.

# Differential fuzzing environment (fuzz/README.md): Debian trixie with clang (the compiler of the
# C++ reference build, crates/cref/build.rs), a nightly Rust toolchain and cargo-fuzz. Built for
# the host platform by fuzz/run.sh.
FROM debian:trixie-slim
RUN apt-get update && apt-get install -y --no-install-recommends \
      clang lld llvm make git python3 binutils curl ca-certificates file pkg-config \
    && rm -rf /var/lib/apt/lists/* \
    && git config --system --add safe.directory '*'
# Nightly Rust (cargo-fuzz needs -Z flags), pinned by NIGHTLY. cargo-fuzz itself is built with a
# stable toolchain: its locked dependencies probe nightly features that current nightlies reject.
ARG NIGHTLY=nightly-2026-10-01
ARG STABLE=1.88.0
ARG CARGO_FUZZ=0.13.1
ENV RUSTUP_HOME=/opt/rustup CARGO_HOME=/opt/cargo PATH=/opt/cargo/bin:$PATH
RUN curl -sSf https://sh.rustup.rs | sh -s -- -y --profile minimal --default-toolchain "$NIGHTLY" \
    && rustup toolchain install --profile minimal "$STABLE" \
    && cargo "+$STABLE" install --locked cargo-fuzz --version "$CARGO_FUZZ" \
    && rustup toolchain uninstall "$STABLE" \
    && rm -rf /opt/cargo/registry /opt/cargo/git \
    && chmod -R a+rwX /opt/rustup /opt/cargo
