#include <string>

/**
 * prepends /home/nao/ to `path`
 * if running locally, prepends $RUNSWIFT_CHECKOUT_DIR/image/home/nao/
 */
std::string getHomeNao(const std::string &path);