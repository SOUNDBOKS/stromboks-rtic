# STROMBOKS Build in RUST

## Setup

Configure rust:

    rustup default nightly
    rustup toolchain install nightly-gnu
    rustup default nightly-gnu
    rustup component add llvm-tools-preview
    cargo install cargo-binutils

Add target to compiler:

    rustup target add thumbv7em-none-eabi

Download and prepare git submodules:

    git submodule update --init --recursive
