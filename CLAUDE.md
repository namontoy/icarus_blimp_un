@AGENTS.md

## Claude Code

AGENTS.md above is shared with every coding agent; put project rules there, not here.

- Skills run as `/core-change`, `/sim-sweep` and `/flash-firmware`; the last runs only when a
  person types it. Their files in `.claude/skills/` are the originals for the Codex copies, so
  after editing one, run `python tools/sync_skills.py`.
- Personal notes go in `CLAUDE.local.md` (gitignored), not in this file.
