#ifndef ESTIMATOR_HPP
#define ESTIMATOR_HPP

class EstimatorInfoInit;
class EstimatorInfoIn;
class EstimatorInfoMiddle;
class EstimatorInfoOut;

class Estimator
{
  public:
    Estimator(const EstimatorInfoInit &estimatorInfoInit)
        : estimatorInfoInit(estimatorInfoInit){}
     virtual ~Estimator() {}

    virtual void tick(
        const EstimatorInfoIn &estimatorInfoIn,
        EstimatorInfoMiddle &estimatorInfoMiddle,
        EstimatorInfoOut &estimatorInfoOut) = 0;
  protected:
    const EstimatorInfoInit &estimatorInfoInit;
};

#endif // ESTIMATOR_HPP
