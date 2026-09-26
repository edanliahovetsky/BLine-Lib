# Contributing

## Release lines

| Branch | Purpose | Vendordep |
| --- | --- | --- |
| `main` | Stable WPILib 2026 development | `BLine-Lib.json` |
| `wpilib-2027` | WPILib 2027 beta development | `BLine-Lib-2027.json` |

Base changes on the intended release line. Keep stable and beta in separate
branches and worktrees; do not merge them as part of repository cleanup.

The library does not need a deployment branch. Each vendordep selects a versioned
Git tag that JitPack builds. Ordinary development commits do not change that
selected artifact. Leave the vendordep pointing at the last released version
until the next release is ready. Installed robot projects retain their selected
version until the team updates the dependency.

For a release, validate the candidate with the matching WPILib and Java versions,
align its version metadata, and publish its immutable version tag. Verify the
tagged JitPack artifacts before making the updated vendordep available, then
publish the reviewed release notes. Keep the existing stable and beta install
URLs unchanged. Preserve release tags; never move a published tag to different
code.

## Local checkouts

Use a durable worktree for the other release line. For example, from a clone with
`main` checked out:

```sh
git fetch origin
git worktree add --track -b wpilib-2027 ../BLine-Lib-2027 origin/wpilib-2027
```

If the local branch already exists, use `git worktree add ../BLine-Lib-2027
wpilib-2027` instead. Track the matching remote branch and use fast-forward
updates when bringing an unchanged checkout current.

Keep short-lived working branches local unless a remote branch is needed for an
agreed contribution. Before deleting one, verify its commits are retained or
preserve them in a verified local backup. Keep uncommitted files and unpublished
experiments, and prune stale registrations only for missing worktrees. Push
explicit branches so local archives and experiments stay local.
