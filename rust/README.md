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

Compile `kuka_kr30l16.cpp` into static library and place into `lib`
