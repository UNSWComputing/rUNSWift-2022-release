# Upgrade instructions

## 3.3.7

```
cd $RUNSWIFT_CHECKOUT_DIR/robot
git rm -r Eigen
tar -xf ~/Downloads/eigen-3.3.7.tar.bz2 eigen-3.3.7/Eigen --strip-components=1
# sed -i 's/DEAD/EIGEN_DEAD/g;' $(rgrep -l DEAD Eigen)
git add Eigen
```
