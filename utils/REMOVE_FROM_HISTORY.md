# separating paths to a new repo

```
PATHS_TO_SEPARATE='^docs$|^research$'

# make a new clone to work in
git clone runswift runswift-rewrite
cd runswift-rewrite/
git remote remove origin
git remote add origin git@github.com:UNSWComputing/rUNSWift
git pull
# add the new remote
git remote add old-docs git@github.com:UNSWComputing/rUNSWift-old-docs.git
# remove non-PATHS from history first
PATHS_TO_REMOVE=$(git log --all --pretty=format: --name-only | sed 's@/.*@@' | sort | uniq | egrep -v "$PATHS_TO_SEPARATE" | xargs)
git filter-branch -f --index-filter "git rm -r --cached --ignore-unmatch $PATHS_TO_REMOVE" --tag-name-filter cat -- --all
# push all branches from old to origin
SRC_REMOTE=origin
DST_REMOTE=old-docs
git push -u $DST_REMOTE $(for a in $(git branch --list --remote "$SRC_REMOTE/*" | grep -v --regexp='->'); do echo "$a:refs/heads/${a//$SRC_REMOTE\/}"; done)
git push -f --tags $DST_REMOTE
cd -
rm -rf runswift-rewrite
```

# clean up the main repo

```
# build-release* was found with the help of https://confluence.atlassian.com/fishkb/determine-the-size-of-each-git-commit-292651328.html
PATHS_TO_REMOVE="docs research build-release robot/tiny-dnn/data robot/tiny-dnn/examples utils/parameter-optimizer/*.txt *.ofn utils/kernels"

# make a new clone to work in
git clone runswift runswift-rewrite
cd runswift-rewrite/
git remote remove origin
git remote add origin git@github.com:UNSWComputing/rUNSWift
git pull
# remove PATHS
git filter-branch -f --index-filter "git rm -r --cached --ignore-unmatch $PATHS_TO_REMOVE" --tag-name-filter cat -- --all
cd -
```

# Updating the cleaned local repo

```
PATHS_TO_REMOVE="docs research build-release robot/tiny-dnn/data robot/tiny-dnn/examples utils/parameter-optimizer/*.txt *.ofn utils/kernels"

cd runswift-rewrite/
git remote update origin
git fetch --tags origin
git filter-branch -f --index-filter "git rm -r --cached --ignore-unmatch $PATHS_TO_REMOVE" --tag-name-filter cat -- --all
cd -
```

# look for large commits

```
wget https://confluence.atlassian.com/fishkb/files/292651328/296583204/1/1342047411459/commit_size.pl
cd runswift-rewrite/
perl $OLDPWD/commit_size.pl | tee /tmp/commit-size.txt
for commit in $(sort -nr /tmp/commit-size.txt | cut -f2 -d' '); do
  if [[ "$(git merge-base $commit master)" == $commit ]]; then
    difftree="$(git diff-tree -r -c -M -C --no-commit-id $commit)"
    for blob in $(cut -f4 -d' ' <<< "$difftree"); do
      [[ "$blob" == 0000000000000000000000000000000000000000 ]] && continue
      size=$(git cat-file --batch-check <<< $blob | cut -f3 -d' ')
      echo -e "$size\t$(grep $blob <<< "$difftree" | cut -f2 -d$'\t')"
    done | sort -n
    echo
    echo "press enter to go to next largest commit or ctrl-c to stop"
    read
  fi
done
cd -
```

# push the clean repo

```
cd runswift-rewrite/
SRC_REMOTE=origin
DST_REMOTE=origin
git push -u $DST_REMOTE $(for a in $(git branch --list --remote "$SRC_REMOTE/*" | grep -v --regexp='->'); do echo "+$a:refs/heads/${a//$SRC_REMOTE\/}"; done)
git push --tags --force
cd -
```

# update your local clone with the clean repo

```
git fetch origin && git checkout origin/master && for branch in $(git branch | cut -c3-); do git branch --delete --force $branch; done && git checkout master
```

# crappy attempt at push hook to prevent this

```
#!/bin/bash
for head_commit in $(cut -f2 -d' '); do
  # somehow figure out the commits to be pushed and their size, and exit 1 if it's too big
done
exit 0
```
