#ifndef FILTERS_H
#define FILTERS_H

#include <iostream>
#include <math.h>
#include <vector>
#include <memory>

//#define DBGMSG 1

namespace wb_controller
{

template <typename T>
inline T secondOrderFilter(T& varOutputSecondFilter , T& varOutputFirstFilter , T const& varNew , T const& gain)
{
    varOutputFirstFilter = (1- gain) * varOutputFirstFilter + gain * varNew;
    varOutputSecondFilter = (1 - gain) * varOutputSecondFilter + gain * varOutputFirstFilter;
    return varOutputSecondFilter;
}

class AvgFilter
{
public:
    typedef std::shared_ptr<AvgFilter> Ptr;

    AvgFilter(unsigned int n=10)
    {
       n_ = n;
       avg_ = 0.0;
    }
    double update(double value)
    {
        avg_ = (avg_ * (n_-1) + value) / n_;
        return avg_;
    }
    void setN(const unsigned int& n)
    {
      assert(n>0);
      n_ = n;
    }
    void reset()
    {
      avg_ = 0.0;
    }

private:
    unsigned int n_;
    double avg_;
};

class MovingAvgFilter
{
public:
  typedef std::shared_ptr<MovingAvgFilter> Ptr;

  MovingAvgFilter(unsigned int n)
  {
    window_.resize(n);
    reset();
  }

  void reset()
  {
    for(unsigned int i=0; i<window_.size(); i++)
      window_[i] = 0.0;
    window_idx_ = 0;
    sum_ = 0.0;
    filter_complete_ = false;
  }

  double update(double value)
  {
      unsigned int n = window_.size();
      double res = 0.0;
      sum_ -= window_[window_idx_];
      sum_ += value;
      window_[window_idx_] = value;
      if (!filter_complete_ && window_idx_ == n - 1) {
          filter_complete_ = true;
        }
        if (filter_complete_) {
          res = sum_ / n;
        } else {
          res = sum_ / (window_idx_+1);
        }
      window_idx_ = (window_idx_+1) % n;
      return res;
  }

private:
  double sum_;
  std::vector<double> window_;
  unsigned int window_idx_;
  bool filter_complete_;
};

namespace filter
{
  /**
    * @brief The bandPassParams struct
    * Data structure for the band-pass filter.
    * BW: bandwidth
    * cf: center frequency
    * sampleFreq: 1/dt
    */
  struct bandPassParams
  {
    double inBW, inCf, inSampleFreq;
    double a0, a1, a2, b1, b2;

    // Default constructor
    bandPassParams() {}
    // Specialized constructor
    bandPassParams(double _BW, double cf, double sampleFreq)
    {
      inBW         = _BW;
      inCf         = cf;
      inSampleFreq = sampleFreq;
      // Compute parameters from BW and f
      double f  = cf / sampleFreq;  // Adjust it from Hz to percentage of sample frequency
      double BW = _BW / sampleFreq; // Adjust it from Hz to percentage of sample frequency
      double R  = 1 - 3*BW;
      double K  = (1 - 2*R*cos(2*M_PI*f)+R*R) / (2 - 2*cos(2*M_PI*f));
#ifdef DBGMSG
      std::cout << "**********************" << std::endl << "Band-pass filter paramers:" << std::endl;
      std::cout << "R: " << R << " K: " << K << std::endl;
#endif

      // Fill in the FIR coefficients
      a0 = 1 - K;
      a1 = 2*(K-R)*cos(2*M_PI*f);
      a2 = R*R - K;
      b1 = 2*R*cos(2*M_PI*f);
      b2 = -R*R;
#ifdef DBGMSG
      std::cout << "a0: " << a0 << " a1: " << a1 << " a2: " << a2 << std::endl;
      std::cout << "b1: " << b1 << " b2: " << b2 << std::endl;
      std::cout << "**********************" << std::endl;
#endif
    }
  };

  /**
    * @brief The bandRejectParams struct
    * Data structure for the band-reject (notch) filter.
    * BW: bandwidth
    * cf: center frequency
    * sampleFreq: 1/dt
    */
  struct bandRejectParams
  {
    double inBW, inCf, inSampleFreq;
    double a0, a1, a2, b1, b2;

    // Default constructor
    bandRejectParams() {}
    // Specialized constructor
    bandRejectParams(double _BW, double cf, double sampleFreq)
    {
      inBW         = _BW;
      inCf         = cf;
      inSampleFreq = sampleFreq;
      // Compute parameters from BW and f
      double f  = cf / sampleFreq;  // Adjust it from Hz to percentage of sample frequency
      double BW = _BW / sampleFreq; // Adjust it from Hz to percentage of sample frequency
      double R  = 1 - 3*BW;
      double K  = (1 - 2*R*cos(2*M_PI*f)+R*R) / (2 - 2*cos(2*M_PI*f));
#ifdef DBGMSG
      std::cout << "**********************" << std::endl << "Notch filter paramers:" << std::endl;
      std::cout << "R: " << R << " K: " << K << std::endl;
#endif

      // Fill in the FIR coefficients
      a0 = K;
      a1 = -2*K*cos(2*M_PI*f);
      a2 = K;
      b1 = 2*R*cos(2*M_PI*f);
      b2 = -R*R;
#ifdef DBGMSG
      std::cout << "a0: " << a0 << " a1: " << a1 << " a2: " << a2 << std::endl;
      std::cout << "b1: " << b1 << " b2: " << b2 << std::endl;
      std::cout << "**********************" << std::endl;
#endif
    }
  };

  /**
    * @brief The lowPassParams struct
    * Data structure for the low-pass filter.
    */
  struct lowPassParams
  {
    double inCutOff, inSampleFreq;
    double alpha;

    // Default constructor
    lowPassParams() {}
    // Specialized constructor
    lowPassParams(double cutOff, double sampleFreq)
    {
      inCutOff    = cutOff;
      inSampleFreq = sampleFreq;
      // Compute parameter alpha from cutoff and sample frequency
      double dt = 1 / sampleFreq;
      double RC = 1 / (inCutOff * 2 * M_PI);
#ifdef DBGMSG
      std::cout << "**********************" << std::endl << "Low-pass filter paramers:" << std::endl;
      std::cout << "RC: " << RC << " dt: " << dt << std::endl;
#endif

      // Fill in the low-pass filter coefficient
      alpha = dt / (RC + dt);
#ifdef DBGMSG
      std::cout << "alpha: " << alpha << std::endl;
      std::cout << "**********************" << std::endl;
#endif
    }
  };

  template <typename T, typename U>
  class band
  {
    public:
      // Default constructor
      band() {}
      // Specialized constructor
      band(U params) : _a0(params.a0), _a1(params.a1), _a2(params.a2), _b1(params.b1), _b2(params.b2),
                       _in_1(0), _in_2(0), _filtered_1(0), _filtered_2(0) {}

      /**
       *  @brief This function visualizes the coefficients from the FIR.
       *  @param none
       */
      void displayCoefficients(void) { std::cout << "A0: " << _a0 << " A1: " << _a1 << " A2: " << _a2 << " B1: " << _b1 << " B2: " << _b2 << std::endl; }

      /**
       *  @brief This function gets the coefficients from the FIR.
       *  @param output a0, a1, a2, b1, b2: coefficients from the FIR
       */
      void getCoefficients(double &a0, double &a1, double &a2, double &b1, double &b2) { a0 = _a0; a1 = _a1; a2 = _a2; b1 = _b1; b2 = _b2; }

      /**
       *  @brief This function computes the band-pass or band-reject (notch) filter offline.
       *  @param input in: input signal
       *  @param input n_samples: number of signal samples
       *  @param input filtered: out filtered signal
       */
      void offlineUpdate(const T *in, T *filtered, int n_samples)
      {
        for (int i = 0; i < n_samples; ++i)
        {
          filtered[i] = _a0 * in[i] + _a1 * _in_1 + _a2 * _in_2 + _b1 * _filtered_1 + _b2 * _filtered_2;
          _in_2 = _in_1;
          _in_1 = in[i];
          _filtered_2 = _filtered_1;
          _filtered_1 = filtered[i];
        }
      }

      /**
       *  @brief This function computes the band-pass or band-reject (notch) filter online.
       *  @param input in: input signal
       *  @param input filtered: out filtered signal
       */
      void onlineUpdate(const T in, T &filtered)
      {
        filtered = _a0 * in + _a1 * _in_1 + _a2 * _in_2 + _b1 * _filtered_1 + _b2 * _filtered_2;
        _in_2 = _in_1;
        _in_1 = in;
        _filtered_2 = _filtered_1;
        _filtered_1 = filtered;
      }

    private:
      double _a0, _a1, _a2, _b1, _b2;
      T      _in_1, _in_2, _filtered_1, _filtered_2;
  };

  template <typename T, typename U>
  class lowPass
  {
    public:
      // Default constructor
      lowPass() {}
      // Specialized constructor
      lowPass(U params) : _alpha(params.alpha), _filtered_1(0) {}

      /**
       *  @brief This function visualizes the coefficient alpha from the low-pass filter.
       *  @param none
       */
      void displayCoefficient(void) { std::cout << "alpha: " << _alpha << std::endl; }

      /**
       *  @brief This function gets the coefficients from the low-pass filter.
       *  @param output alpha: coefficient from the low-pass filter
       */
      void getCoefficients(double &alpha) { alpha = _alpha; }

      /**
       *  @brief This function computes the low-pass filter offline.
       *  @param input in: input signal
       *  @param input n_samples: number of signal samples
       *  @param input filtered: out filtered signal
       */
      void offlineUpdate(const T *in, T *filtered, int n_samples)
      {
        for (int i = 0; i < n_samples; ++i)
        {
          filtered[i] = (1 - _alpha) * _filtered_1 + _alpha * in[i];
          _filtered_1 = filtered[i];
        }
      }

      /**
       *  @brief This function computes the low-pass filter online.
       *  @param input in: input signal
       *  @param input params: parameters for the band filter
       *  @param input filtered: out filtered signal
       */
      void onlineUpdate(const T in, T &filtered)
      {
        filtered    = (1 - _alpha) * _filtered_1 + _alpha * in;
        _filtered_1 = filtered;
      }

    private:
      double _alpha;
      T _filtered_1;
  };
} // end namespace filter
} // end namespace wb_controller

#endif // FILTERS_H
