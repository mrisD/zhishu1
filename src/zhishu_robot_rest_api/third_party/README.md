# Vendored dependencies

These header-only dependencies are stored in the package to make clean builds
independent of external network availability.

- `nlohmann_json`: 3.12.0, MIT, `nlohmann_json/LICENSE.MIT`
- `cpp_httplib`: 0.46.0, MIT, `cpp_httplib/LICENSE`

When updating either dependency, replace the header and its license together,
then run the package build and tests from a clean build directory.
