# Linear Subproblem Solutions (Rust)

## Usage

1. Clone the rust directory

2. Add the following to your project's `Cargo.toml`

    ```
    [dependencies.linear-subproblem-solutions-rust]
        path = ".../linear-subproblem-solutions/rust"
    ```

3. Add to your source files where needed

    ```
    use linear_subproblem_solutions_rust as subproblems;
    use subproblems::nalgebra as na;
    ```

### Testing

#### Correctness Tests

Run `cargo test correctness`

Run `cargo test correctness -- --nocapture` for diagnostic info

#### Timing Tests

Run `cargo test timing --release -- --nocapture --test-threads 1`

### Benchmarking

Run `cargo bench`

### Linking ikfast

Compile `kuka_kr30l16.cpp` into a static library and place it into `lib`

Example compilation for msvc toolchain:

`cl /O2 /c /EHsc /Folib/kuka_kr30l16.obj /DIKFAST_NO_MAIN ikfast/kuka_kr30l16.cpp`
`lib lib/kuka_kr30l16.obj /out:lib/kuka_kr30l16.lib`
