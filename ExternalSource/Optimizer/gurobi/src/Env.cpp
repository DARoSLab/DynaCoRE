// Copyright (C) 2016, Gurobi Optimization, Inc.
// All Rights Reserved

#include "Common.h"
#include "parprivate.h"

GRBEnv::GRBEnv(GRBenv* Cenv)
{
  env = Cenv;
  envP = NULL;
}

GRBEnv::GRBEnv()
{
  int error = GRBloadenv(&env, NULL);
  if (error) throw GRBException(getErrorMsg(), error);
  envP = &env;
}

GRBEnv::GRBEnv(const string& logfilename)
{
  int error = GRBloadenv(&env, logfilename.c_str());
  if (error) throw GRBException(getErrorMsg(), error);
  envP = &env;
}

GRBEnv::GRBEnv(const string& logfilename,
               const string& computeserver,
               int           port,
               const string& password,
               int           priority,
               double        timeout)
{
  int error = GRBloadclientenv(&env, logfilename.c_str(), computeserver.c_str(),
                               port, password.c_str(), priority, timeout);
  if (error) throw GRBException(getErrorMsg(), error);
  envP = &env;
}

GRBEnv::~GRBEnv()
{
  if (envP != NULL && envP == &env) {
    GRBfreeenv(env);
    env = NULL;
  }
}

void
GRBEnv::message(const string& msg)
{
  GRBmsg(env, msg.c_str());
}

int
GRBEnv::get(GRB_IntParam param) const
{
  int value;
  int error = GRBgetintparam(env, iparname[param], &value);
  if (error) throw GRBException(getErrorMsg(), error);
  return value;
}

double
GRBEnv::get(GRB_DoubleParam param) const
{
  double value;
  int error = GRBgetdblparam(env, dparname[param], &value);
  if (error) throw GRBException(getErrorMsg(), error);
  return value;
}

const string
GRBEnv::get(GRB_StringParam param) const
{
  char value[GRB_MAX_STRLEN];
  int error = GRBgetstrparam(env, sparname[param], value);
  if (error) throw GRBException(getErrorMsg(), error);
  return string(value);
}

void
GRBEnv::set(GRB_IntParam param, int newvalue)
{
  int error = GRBsetintparam(env, iparname[param], newvalue);
  if (error) throw GRBException(getErrorMsg(), error);
}

void
GRBEnv::set(GRB_DoubleParam param, double newvalue)
{
  int error = GRBsetdblparam(env, dparname[param], newvalue);
  if (error) throw GRBException(getErrorMsg(), error);
}

void
GRBEnv::set(GRB_StringParam param, const string& newvalue)
{
  int error = GRBsetstrparam(env, sparname[param], newvalue.c_str());
  if (error) throw GRBException(getErrorMsg(), error);
}

void
GRBEnv::set(const string& paramname, const string& newvalue)
{
  int error = GRBsetparam(env, paramname.c_str(), newvalue.c_str());
  if (error) throw GRBException(getErrorMsg(), error);
}

void
GRBEnv::getParamInfo(GRB_DoubleParam param, double* valP,
                     double* minP, double* maxP, double* defP)
{
  int error = GRBgetdblparaminfo(env, dparname[param], valP, minP,
                                 maxP, defP);
  if (error) throw GRBException(getErrorMsg(), error);
}

void
GRBEnv::getParamInfo(GRB_IntParam param, int* valP, int* minP,
                     int* maxP, int* defP)
{
  int error = GRBgetintparaminfo(env, iparname[param], valP, minP,
                                 maxP, defP);
  if (error) throw GRBException(getErrorMsg(), error);
}

void
GRBEnv::getParamInfo(GRB_StringParam param, string& value,
                     string& defvalue)
{
  char cvalue[GRB_MAX_STRLEN];
  char cdefvalue[GRB_MAX_STRLEN];
  int error = GRBgetstrparaminfo(env, sparname[param],
                                 cvalue, cdefvalue);
  if (error) throw GRBException(getErrorMsg(), error);
  value    = string(cvalue);
  defvalue = string(cdefvalue);
}

void
GRBEnv::resetParams()
{
  int error = GRBresetparams(env);
  if (error) throw GRBException(getErrorMsg(), error);
}

void
GRBEnv::writeParams(const string& paramfile)
{
  int error = GRBwriteparams(env, paramfile.c_str());
  if (error) throw GRBException(getErrorMsg(), error);
}

void
GRBEnv::readParams(const string& paramfile)
{
  int error = GRBreadparams(env, paramfile.c_str());
  if (error) throw GRBException(getErrorMsg(), error);
}

const string
GRBEnv::getErrorMsg() const
{
  const char* str = GRBgeterrormsg(env);
  return string(str);
}
