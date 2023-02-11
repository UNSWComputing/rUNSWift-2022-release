//
// Created by jayen on 23/2/20.
//

#include "utils/home_nao.hpp"

#include <cstdlib>

using std::string;

string getHomeNao(const string &path) {
   char         *runswiftCheckoutDir = getenv("RUNSWIFT_CHECKOUT_DIR");
   const string homeNaoPath          = "/home/nao/" + path;
   if (runswiftCheckoutDir == NULL) {
      return homeNaoPath;
   } else {
      return runswiftCheckoutDir + ("/image" + homeNaoPath);
   }
}
