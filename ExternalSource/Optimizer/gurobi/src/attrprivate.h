// Copyright (C) 2016, Gurobi Optimization, Inc.
// All Rights Reserved
#ifndef _ATTRPRIVATE_CPP_H_
#define _ATTRPRIVATE_CPP_H_

// orders need to match enums in public.h

static const char* iattrname[] = {
  "NumConstrs",
  "NumVars",
  "NumSOS",
  "NumQConstrs",
  "NumNZs",
  "NumQNZs",
  "NumQCNZs",
  "NumIntVars",
  "NumBinVars",
  "NumPWLObjVars",
  "ModelSense",
  "IsMIP",
  "IsQP",
  "IsQCP",
  "Status",
  "SolCount",
  "BarIterCount",
  "VBasis",
  "CBasis",
  "PWLObjCvx",
  "BranchPriority",
  "VarPreStat",
  "BoundVioIndex",
  "BoundSVioIndex",
  "ConstrVioIndex",
  "ConstrSVioIndex",
  "ConstrResidualIndex",
  "ConstrSResidualIndex",
  "DualVioIndex",
  "DualSVioIndex",
  "DualResidualIndex",
  "DualSResidualIndex",
  "ComplVioIndex",
  "IntVioIndex",
  "IISMinimal",
  "IISLB",
  "IISUB",
  "IISConstr",
  "IISSOS",
  "IISQConstr",
  "TuneResultCount",
  "Lazy",
  "VarHintPri"
};

static const char* cattrname[] = {
  "VType",
  "Sense",
  "QCSense"
};

static const char* dattrname[] = {
  "Runtime",
  "ObjCon",
  "LB",
  "UB",
  "Obj",
  "Start",
  "PreFixVal",
  "RHS",
  "QCRHS",
  "MaxCoeff",
  "MinCoeff",
  "MaxBound",
  "MinBound",
  "MaxObjCoeff",
  "MinObjCoeff",
  "MaxRHS",
  "MinRHS",
  "ObjVal",
  "ObjBound",
  "ObjBoundC",
  "MIPGap",
  "IterCount",
  "NodeCount",
  "X",
  "RC",
  "Pi",
  "QCPi",
  "Slack",
  "QCSlack",
  "BoundVio",
  "BoundSVio",
  "BoundVioSum",
  "BoundSVioSum",
  "ConstrVio",
  "ConstrSVio",
  "ConstrVioSum",
  "ConstrSVioSum",
  "ConstrResidual",
  "ConstrSResidual",
  "ConstrResidualSum",
  "ConstrSResidualSum",
  "DualVio",
  "DualSVio",
  "DualVioSum",
  "DualSVioSum",
  "DualResidual",
  "DualSResidual",
  "DualResidualSum",
  "DualSResidualSum",
  "ComplVio",
  "ComplVioSum",
  "IntVio",
  "IntVioSum",
  "Kappa",
  "KappaExact",
  "SAObjLow",
  "SAObjUp",
  "SALBLow",
  "SALBUp",
  "SARHSLow",
  "SAUBLow",
  "SAUBUp",
  "SARHSUp",
  "Xn",
  "FarkasProof",
  "FarkasDual",
  "UnbdRay",
  "PStart",
  "DStart",
  "BarX",
  "VarHintVal"
};

static const char* sattrname[] = {
  "ModelName",
  "VarName",
  "ConstrName",
  "QCName"
};

static void checkattrsize(GRBmodel*   Cmodel,
                          const char* attrname,
                          int         size)
{
  int i;
  int error = GRBgetattrinfo(Cmodel, attrname, NULL, &i, NULL);
  if (error == 0 && i != size) error = GRB_ERROR_INVALID_ARGUMENT;
  if (error) throw GRBException("Not right attribute", error);
}
#endif
