## 2026-02-14 - Vehicle Projection Cache Updating
**Learning:** When caching computed attributes like distance projections (`v.proj`) to accelerate sorting, they must be strictly updated whenever the dependencies (e.g. `v.position`) change, specifically at the start of each simulation tick. Leaving them stale results in corrupt binary searches and failed leader lookups.
**Action:** Always verify that cached properties on state structs are kept in sync within the core state-mutation methods of the tick loop.
