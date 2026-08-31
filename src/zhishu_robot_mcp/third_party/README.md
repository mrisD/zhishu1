# Vendored dependencies

`cxxmcp/` contains the complete cxxmcp SDK v1.2.8 release source used by this
package. It is built directly with `add_subdirectory`, so clean builds do not
need network access.

- Upstream: `https://github.com/caomengxuan666/cxxmcp`
- Release artifact: `cxxmcp-sdk-source-v1.2.8.tar.gz`
- SHA-256: `ac4be3cd7ba6938960ff48cf25f7ea3293909df60bf7d79bb44d464b69499c29`
- License: MIT; see `cxxmcp/LICENSE`

The upstream release artifact omits `VERSION`; this directory restores it as
`1.2.8`, matching the pinned and checksum-verified release.
