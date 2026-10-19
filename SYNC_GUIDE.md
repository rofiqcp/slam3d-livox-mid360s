# Git synchronization: slam3d-livox-mid360s

Repository: https://github.com/rofiqcp/slam3d-livox-mid360s

- Sirobo: /media/sirobo/HDD/Project/slam3d-livox-mid360s
- MiniPC1: /home/otomasi/Project/slam3d-livox-mid360s
- Shared default branch: v1

Before editing:

    git switch v1
    git pull --ff-only origin v1

After editing and reviewing changes:

    git status --short
    git add <paths-to-source-and-docs>
    git commit -m "Describe the change"
    git push origin v1

Run git pull --ff-only origin v1 on the other computer.
Captured data, ROS builds, logs, backups, credentials and local host
artifacts are deliberately not synchronized. Avoid simultaneous edits
to the same files without pulling first.
