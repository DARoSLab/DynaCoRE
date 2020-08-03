// Copyright (C) 2016, Gurobi Optimization, Inc.
// All Rights Reserved
#ifndef _ENV_CPP_H_
#define _ENV_CPP_H_

class GRBEnv
{
  private:

    GRBenv*  env;
    GRBenv** envP;

// only for gurobi_c++.h const GRBEnv& operator=(const GRBEnv &xenv);

    GRBEnv(GRBenv *Cenv);

  public:

    friend class GRBModel;

    GRBEnv();
    GRBEnv(const string& logfilename);
    GRBEnv(const string& logfilename, const string& computeservers, int port,
           const string& password, int priority, double timeout);
    GRBEnv(const string&, const string&, const string&, int, const string&);
    ~GRBEnv();
    void message(const string& msg);
    int get(GRB_IntParam param) const;
    double get(GRB_DoubleParam param) const;
    const string get(GRB_StringParam param) const;
    void set(GRB_IntParam param, int newvalue);
    void set(GRB_DoubleParam param, double newvalue);
    void set(GRB_StringParam param, const string& newvalue);
    void set(const string& paramname, const string& newvalue);
    void getParamInfo(GRB_DoubleParam param, double* valP,
                      double* minP, double* maxP, double* defP);
    void getParamInfo(GRB_IntParam param, int* valP, int* minP,
                      int* maxP, int* defP);
    void getParamInfo(GRB_StringParam param, string& value,
                      string& defvalue);
    void resetParams();
    void writeParams(const string& paramfile);
    void readParams(const string& paramfile);
    const string getErrorMsg() const;
};
#endif
