# AGENTS.md

Rules for people and coding agents working in this repository.

## Owned repository contract

This repository is the user's public, independently maintained copy [paulnamalomba/seskaocct](https://github.com/paulnamalomba/seskaocct) of Open CASCADE Technology (OCCT). It was renamed from `paulnamalomba/OCCT` on 10 November 2026 and detached from the GitHub fork network, so it is a standalone repository. SESKAFEA's frontend geometry workers (kernel, STEP and IGES translation) build OCCT from an exact commit of this repository. The commit is pinned with the SHA-256 of its GitHub archive in the frontend's `cmake/FrontendDependencies.json`. All intentional OCCT source changes for SESKAFEA belong here as reviewed commits, never as patch files hidden in consumer build scripts. Publish a commit here before a consumer pins it.

**Required (user, 3 November 2026; renamed and detached 10 November 2026):** this repository is public and independently maintained, and upstream `Open-Cascade-SAS/OCCT` is a read/fetch reference only:

- Do not configure a push URL for upstream.
- Do not push branches or tags upstream, and do not open pull requests, issues or comments there.
- This is an independently maintained copy with upstream as a read/fetch reference. Every push and every pull request must explicitly target `paulnamalomba/seskaocct`. Create pull requests only when the user asks or the [agent workflow](#agent-workflow) below requires one.

### Branches

- `native/v8.0.1` starts from upstream tag `V8_0_1` (`b8f597c677811d1f9f4d8a97f5ae2825c0353a42`, 2026-07-30), whose source archive has SHA-256 `dba62b81078dd43cec23feba89432be301582341001edad1b93342ad8bda35ea`. It carries the SESKAFEA changes as separate commits.
- Changes land on `native/v8.0.1`, the product branch that the suite pins. Pull requests target it, and `main` follows it.
- A later upstream release gets a new `native/v<release>` branch. Each change is re-applied and re-qualified there; a version number never implies compatibility.
- Never rewrite or force-push a commit that a consumer has pinned.

### Change rules

- **Required:** one reviewed change per commit. The message states the defect, its source location, the remedy and the evidence that tested both the raw defect and the remedy.
- **Required:** keep each change minimal and local. Preserve OCCT's public API and ABI unless the user decides otherwise, and keep upstream naming and style in OCCT sources.
- **Required:** a change is adopted only after it has an independent review and the consumer's acceptance gates pass on the fork commit. A zero exit is never success evidence by itself.
- Keep upstream's version macros (`OCC_VERSION_*`) unchanged. The internal version below is separate.

### Internal Version And Build Records

**Required (user, 10 November 2026):** owned dependency forks follow the established fork practice of the [MS-MPI fork](https://github.com/paulnamalomba/microsoft-mpi-native-windows). This section applies that practice to this fork.

`OCCT_INTERNAL_VERSION` is the fork's authoritative internal four-part version, independent of upstream's `OCC_VERSION_*` macros:

```text
<breaking>.<major>.<minor>.<build-number>
```

- The seed is `8.0.1.0`: upstream release 8.0.1 plus build zero. It is unbuilt, and it is not a release or a passing-build claim. The first qualifying build advances it to `8.0.1.1`. Build zero after a later rollover can identify a successful build.
- Major and minor each range from 0 through 9. **Required (user, 17 November 2026):** the build field ranges from 0 through 99 without zero padding, as in every owned repository; a build field of 100 is refused. Fields are ASCII digits only (`0`–`9`; other Unicode digits are refused), and a version value has no trailing newline or other surrounding whitespace; a reader of the version file strips the file's line end before it parses. A successful build increments the fourth field: `8.0.1.9 -> 8.0.1.10`. Overflow carries to the next field and resets the following fields to zero: `8.0.1.99 -> 8.0.2.0`, `8.0.9.99 -> 8.1.0.0`, `8.9.9.99 -> 9.0.0.0`. This supersedes the earlier one-digit build field; no build was accepted under it. Coordinate allocation across branches, and never publish different artifacts under the same complete identity.
- A build qualifies when the consumer's acceptance builds this fork commit from its pinned archive and its required checks pass. Record the actual scope. The per-PR focused evidence (the incremental build, the focused checks and the council review) qualifies a merged PR for its own build number and focused build record; full acceptance stays at the family or batch run and remains open until then. Failed, cancelled, skipped, configure-only, documentation-only and no-op work does not advance any field.
- Explicit breaking, major or minor decisions select a candidate family. They do not change `OCCT_INTERNAL_VERSION` until an accepted build confirms that candidate.
- Do not encode the internal identity into upstream's `OCC_VERSION_*` macros.
- Group notes by the first three fields in Markdown: `.commits/<breaking>.<major>.<minor>.md`, so the 8.0.1 family shares [.commits/8.0.1.md](.commits/8.0.1.md). Use the `create-commit-notes` skill at seed initialization and at every minor, major or breaking change, including carries. Append every accepted build. Never collapse or rewrite earlier transitions.
- Each accepted build gets `.commits/builds/<version>.json` and a family-note record. The build record holds `accepted`, `status`, `source_commit` (the 40-character fork commit that the consumer built), checks with results, artifact or archive hashes and remaining limits. Persist the accepted version and its record in the same change. This record layout is Proposed.
- `.commits/manage-repo.ps1` reads `OCCT_INTERNAL_VERSION` and never changes it. Any `-Version` must equal it. Mutating actions require an attached branch, a clean tree and `-Yes`. Pushes target only `paulnamalomba/seskaocct`. Tags use the `native/v<internal version>` namespace and are never replaced. Run `pwsh -NoProfile -File .commits/manage-repo.ps1 -Action check` for its self-checks, and add `-WhatIf` to preview a mutating action.
### Change register

| Commit subject | Files | Evidence |
| --- | --- | --- |
| Throw from AllocateOptimal when malloc fails | `TKernel/Standard/Standard.cxx` | SESKAFEA X1 checked-allocation controls; frontend C1.0c pin |
| Return null for null STEP vectors and directions | `TKDESTEP/StepToGeom/StepToGeom.cxx` | SESKAFEA X3 P1 (inline-vector `0xC0000005`) |
| Widen the STEP and IGES real write buffers | `StepData_StepWriter.cxx`, `IGESData_IGESWriter.cxx` | SESKAFEA X3 P2 (`0xC0000409` on 17-digit reals) |
| Bound Interface_FloatWriter formatting | `TKXSBase/Interface/Interface_FloatWriter.cxx` | SESKAFEA X3 P3 (truncated zero suppression, format overflow) |
| Pass Standard_OutOfMemory out of BRepMesh and flag face meshing failures | `TKMesh/BRepMesh/*` (7 files), `TKMesh/IMeshTools/IMeshTools_Model{Algo,Builder}.hxx`, GTest `BRepMesh_FailurePropagation_Test.cxx` | Issue #5; SESKAFEA C1.0d `fixture.mesh-sequence`; GTest 5 fail unpatched, 32/32 pass patched |
| Keep Standard_OutOfMemory typed across OSD_ThreadPool | `TKernel/OSD/OSD_ThreadPool.{cxx,hxx}`, GTest `OSD_Parallel_Test.cxx` | Issue #5; 4 new GTests, 4 fail with the change stashed |
| Report BRepMesh failures in the mesher status and define the state after out of memory | `TKMesh/BRepMesh/*` (7 files), `TKMesh/IMeshTools/IMeshTools_{MeshBuilder.cxx,ModelAlgo.hxx,ModelBuilder.hxx}`, GTest `BRepMesh_FailurePropagation_Test.cxx` | Issue #5 review; 13 fail with the change stashed, 49/49 pass (`BRepMesh_*:OSD_Parallel*`) |
| Keep the thread pool alive when storing an out-of-memory failure fails | `TKernel/OSD/OSD_ThreadPool.cxx`, GTest `OSD_Parallel_Test.cxx` | Issue #5 review round 2; capture path argued from code, precedence mutation fails 1 test; `sizeof` of `EnumeratedThread` 432 and `Launcher` 480 unchanged |
| Restore the previous mesh when BRepMesh_IncrementalMesh runs out of memory | `TKMesh/BRepMesh/*` (7 files), `TKMesh/IMeshTools/IMeshTools_{Context.hxx,MeshBuilder.cxx}`, `TKBRep/BRep/BRep_TFace.hxx`, GTest `BRepMesh_FailurePropagation_Test.cxx` | Issue #5 review round 2; 11 mutations each fail their test, 59/59 pass (`BRepMesh_*:OSD_Parallel*`) |
| Count every failed thread when the thread pool reports out of memory | `TKernel/OSD/OSD_ThreadPool.cxx`, GTest `OSD_Parallel_Test.cxx` | Issue #5 review round 3; count mutation fails 1 new test, 61/61 pass (`BRepMesh_*:OSD_Parallel*`) |
| Save the mesh before the BRepMesh run, apart from its restore | `TKMesh/BRepMesh/BRepMesh_{IncrementalMesh,FaceDiscret}.cxx`, GTest `BRepMesh_FailurePropagation_Test.cxx` | Issue #5 review round 3; new test fails on 4ec149b86 code, 61/61 pass (`BRepMesh_*:OSD_Parallel*`) |

## Agent workflow

These rules apply to the orchestrator and its subagents. The suite `AGENTS.md` holds the same common rules, and every owned repository repeats them. Changes land on the product branch `native/v8.0.1`, whose commit the suite pins; pull requests target it and `main` follows it. Below, "the default branch" means `native/v8.0.1`. The [owned repository contract](#owned-repository-contract) above stays in force: upstream is read-only, every pull request targets `paulnamalomba/seskaocct`, and OCCT sources keep upstream naming and style.

- **Required (user, 16 November 2026), main stays current:** once a pull request merges, its work must be on the default branch. When a PR merges into an intermediate or stacked branch, a merge agent carries that branch into the default branch at once: it merges with `--no-ff`, resolves conflicts by intent, verifies, pushes, and merges the merge PR under the user's standing authorisation for this step. Agents never merge a PR the user has not merged. Open stacked PRs against the default branch where possible, because `Closes #n` links and closes issues only for PRs into the default branch.
- **Required (user, 16 November 2026), subagent models:** implementers and clerks run Claude Fable 5.2 at low effort; reviewers and council seats run Claude Opus 5.5 at high effort (owner, 17 November 2026, replacing the 16 November assignments). The orchestrator works as chief of staff: it hands work to background subagents through context pointers (brief files, commits, issues) and improves the agent environment before feature work, with no workarounds.
- **Required (user, 16 November 2026), skill routing:** use `planning-and-task-breakdown` for features and new implementations (the broad points of the implementation roadmap) before issues are written, `multi-opinion` for architecture decisions and for `research`, and `issue-and-pr-based-development` for the tracker, branch and PR flow.
- **Required (user, 17 November 2026), work per issue or PR:** each issue or PR gets an incremental build of the whole tree with no warnings, focused tests for the modules, gates and fixtures the change touches, mutation evidence that each new test catches its defect, and a council review. Then it merges. Here each PR runs the fork's own focused checks (the GTests and tests of the touched toolkits, built incrementally in the fork worktree) plus the consumer's incremental build and focused geometry tests on the fork commit. A PR runs no full suite and no build tier, and nothing is rerun after a clean merge of the default branch. This supersedes the 16 November "test scope per step" rule and the 11 November per-build tier rule, and the rebuild sentence of the merge verification reuse rule for clean merges of the default branch.
- **Required (user, 17 November 2026), build number per merged PR:** each merged PR that changes source, tests, build files, tools or dependencies gets its own build number and a build record that holds its focused evidence: the incremental build, the focused tests, the mutation evidence and the council review. A PR that changes no build-affecting file (only docs, notes or evidence) takes no build number, with the `git diff --name-only` output as evidence; documentation-only work does not advance a build number. This fork allocates the number under [Internal Version And Build Records](#internal-version-and-build-records): only a qualifying build advances the counter, and the build field runs 0–99. A change to tools only produces no artifact build of its own: it takes no number at merge and rides on the next qualifying build, because the counter advances only on a qualifying artifact build.
- **Required (user, 17 November 2026), phase-end full run:** the consumer's full acceptance gates on the fork commit and the build or acceptance run happen once per batch, after the implementation of the phase is done, on the batch head. Their failures are fixed then. The record of that run lists the PR build numbers it covers. Every full run keeps its complete logs. This rule saves time only. It does not relax any numerical or acceptance check at the phase-end run.
- **Required (user, 17 November 2026), build trees:** a build tree is kept until its branch merges and is rebuilt incrementally.
- **Required (user, 17 November 2026), council review per PR:** every PR gets a council review before it merges, with scores out of 10 and a "Council review" table in the PR body. A normal PR is reviewed by one Claude Opus 5.5 (high) and one Gemini 3.8 Flash (High). A big or risky PR is reviewed by one Claude Opus 5.5 (high) and two Gemini 3.8 Flash (High); big or risky means a vendor or fork patch (any change to OCCT source in this repository), a whole package, a storage-format change, or a solver or numerics change. This replaces the 11 and 12 November rule of one independent code review per PR. Issue acceptance criteria plus tests remain the contract, with no prose contract freezes. The council review is the independent review required under [Change rules](#change-rules).
- **Required (user, 17 November 2026), research council:** a research council is three Gemini 3.8 Flash (High) and one Claude Opus 5.5 (high). Decision councils stay as before.
- **Required (user, 16 November 2026), merge verification reuse:** a merge that changes no build-affecting file (only verification evidence, notes or docs) reuses the verified parent's results, with the `git diff --name-only` output as evidence. Any change to source, tests, build files, tools or dependencies is rebuilt and verified.

## Shared instruction entry points

`CLAUDE.md`, `GEMINI.md` and `ANTIGRAVITY_INSTRUCTIONS.md` reference `@AGENTS.md`; keep substantive rules here. Upstream's README and coding rules remain in force except where this file supersedes them.
