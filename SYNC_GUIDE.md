# Synchronize `slam3d-livox-mid360s` on two Linux PCs

- GitHub: https://github.com/rofiqcp/slam3d-livox-mid360s (public)
- Branch: `v1` (default)
- Sirobo: `/media/sirobo/HDD/Project/slam3d-livox-mid360s`
- MiniPC1: `/home/otomasi/Project/slam3d-livox-mid360s`

Before editing:

```bash
git status --short
git switch v1
git pull --ff-only origin v1
```

After reviewing changes:

```bash
python3 tools/check_repo_hygiene.py
git status --short
git add src/ config/ tools/ docs/ README.md AGENTS.md ROADMAP.md   # only if changed
git diff --cached --stat
git commit -m "Describe the reviewed change"
git push origin v1
```

On the other machine run `git pull --ff-only origin v1`. Local maps, bag files, point-cloud samples, ROS outputs and research archives are **not** synchronized through Git. If the working tree has local modifications, preserve or commit them before pulling; never force/reset without inspecting what would be lost.
