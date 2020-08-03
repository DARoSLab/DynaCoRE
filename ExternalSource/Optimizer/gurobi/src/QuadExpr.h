// Copyright (C) 2016, Gurobi Optimization, Inc.
// All Rights Reserved
#ifndef _QUADEXPR_CPP_H
#define _QUADEXPR_CPP_H

class GRBQuadExpr: public GRBExpr
{
  private:

    GRBLinExpr linexpr;
    vector<double> coeffs;
    vector<GRBVar> vars1;
    vector<GRBVar> vars2;
    void multAdd(double m, const GRBQuadExpr& expr);

  public:

    GRBQuadExpr(double constant=0.0);
    GRBQuadExpr(GRBVar var, double coeff=1.0);
    GRBQuadExpr(GRBLinExpr le);

    friend ostream& operator<<(ostream &stream, GRBQuadExpr expr);
    friend GRBQuadExpr operator+(const GRBQuadExpr& x, const GRBQuadExpr& y);
    friend GRBQuadExpr operator-(const GRBQuadExpr& x, const GRBQuadExpr& y);
    friend GRBQuadExpr operator+(const GRBQuadExpr& x);
    friend GRBQuadExpr operator-(const GRBQuadExpr& x);
    friend GRBQuadExpr operator*(const GRBQuadExpr& x, double a);
    friend GRBQuadExpr operator*(double a, const GRBQuadExpr& x);
    friend GRBQuadExpr operator*(GRBVar x, GRBVar y);
    friend GRBQuadExpr operator*(GRBVar x, const GRBLinExpr& y);
    friend GRBQuadExpr operator*(const GRBLinExpr& y, GRBVar x);
    friend GRBQuadExpr operator*(const GRBLinExpr& x, const GRBLinExpr& y);
    friend GRBQuadExpr operator/(const GRBQuadExpr& x, double a);

    unsigned int size(void) const;
    GRBVar getVar1(int i) const;
    GRBVar getVar2(int i) const;
    double getCoeff(int i) const;
    GRBLinExpr getLinExpr() const;
    double getValue() const;

    void addConstant(double c);
    void addTerm(double coeff, GRBVar var);
    void addTerm(double coeff, GRBVar var1, GRBVar var2);
    void addTerms(const double* coeff, const GRBVar* var, int cnt);
    void addTerms(const double* coeff, const GRBVar* var1,
                  const GRBVar* var2, int cnt);
    void add(const GRBLinExpr le);
    GRBQuadExpr operator=(const GRBQuadExpr& rhs);
    void operator+=(const GRBQuadExpr& expr);
    void operator-=(const GRBQuadExpr& expr);
    void operator*=(double mult);
    void operator/=(double a);
    GRBQuadExpr operator+(const GRBQuadExpr& rhs);
    GRBQuadExpr operator-(const GRBQuadExpr& rhs);
    void remove(int i);
    bool remove(GRBVar v);

    void clear();
};
#endif
