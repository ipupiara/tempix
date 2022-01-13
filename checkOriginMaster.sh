echo "namestatus: "
git diff --name-status master origin/master
echo " "
echo "git local HEAD commit ID to be checked with github.com below from SSH connection"
echo "local:"
echo "======   "
git rev-parse --short HEAD
git show-ref --head
echo "..shortstat HEAD: "
git show --shortstat HEAD
echo "   "
echo "remote: "
echo "======="
git ls-remote origin
echo "   "
echo "ATTENTION current Head on remote origin, tobe checked with above local head"
echo "if all id's are the same, it is anyhow ok to push "
