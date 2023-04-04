# Linear Subproblem Solutions (Rust)

## Usage

1. Clone the rust directory

2. Add the following to your project's `Cargo.toml`

    ```rust
    [dependencies.linear-subproblem-solutions-rust]
        path = ".../linear-subproblem-solutions/rust"
    ```

3. Add to your source files where needed

    ```rust
    use linear_subproblem_solutions_rust as subproblems;
    use subproblems::nalgebra as na;
    ```

### Testing

#### Correctness Tests

```shell
$ cargo test correctness
```

For diagnostic info:

```shell
$ cargo test correctness -- --nocapture
```

#### Timing Tests

```shell
$ cargo test timing --release -- --nocapture --test-threads 1
```

### Benchmarking

```shell
$ cargo bench
```

### Linking ikfast

Compile `kuka_kr30l16.cpp` into a static library and place it into a folder called `lib` in the project directory.

Example compilation for the msvc toolchain:

```shell
$ cl /O2 /c /EHsc /Folib/kuka_kr30l16.obj /DIKFAST_NO_MAIN ikfast/kuka_kr30l16.cpp
$ lib lib/kuka_kr30l16.obj /out:lib/kuka_kr30l16.lib
```

Example compilation for the gnu toolchain:

```shell
$ g++ -O3 -o lib/kuka_kr30l16.obj -DIKFAST_NO_MAIN ikfast/kuka_kr30l16.cpp
$ ar rcs lib/kuka_kr30l16.a lib/kuka_kr30l16.obj
```
