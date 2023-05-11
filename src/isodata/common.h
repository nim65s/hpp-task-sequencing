//
// Created by Jeff on 2019/1/6 0006.
//

#ifndef ISODATA_COMMON_H
#define ISODATA_COMMON_H

#include <vector>
#include <iostream>
#include <cmath>
#include "error.h"

using namespace std;

template <typename T>
double get_distance(vector<T> &p1, vector<T> &p2); // add 'double k' to the params for hole distance
template <typename T>
vector<T> operator+(vector<T> &left, vector<T>& right);
template <typename T>
vector<T> operator+(vector<T> left, vector<T> right);
template <typename T>
void operator+=(vector<T> &left, vector<T>& right);
template <typename T1, typename T2>
vector<double> operator/(vector<T1> &left, T2 r);
template <typename T1, typename T2>
vector<double> operator/(vector<T1> &&left, T2 r);
template <typename T>
void operator/=(vector<T> &left, double r);
template <typename T>
vector<double> mypow(vector<T> &left, double p);
template <typename T>
vector<double> mypow(vector<T> &&left, double p);
template <typename T>
vector<T> operator-(vector<T> &left, vector<T> &right);
template <typename T>
void operator+=(vector<T>& left, vector<T>&& right);
template <typename T>
vector<double> vector_sqrt(vector<T>& left);
template <typename T>
vector<double> operator*(vector<double> &left, T d);
template <typename T>
ostream& operator<<(ostream& out, vector<T>& tvec);



// // returns the euclidean distance between points p1 and p2
// template <typename T>
// double get_distance(vector<T> &p1, vector<T> &p2)
// {
//     if (p1.size() != p2.size()){
//       cout << WARN_VECTOR_SIZE << endl;
//       return -1;
//     }
//     else{
//         double res(0);
//         for (int i = 0; i < p1.size(); ++i) {
//             res += pow(p1[i]-p2[i], 2);
//         }
//         return sqrt(res);
//     }
// }

// returns the distance between hole configurations p1 and p2
template <typename T>
double get_distance(vector<T> &p1, vector<T> &p2) // add 'double k' for hole distance
{
  if (p1.size() != 7 || p2.size() != 7) {
    cout << WARN_VECTOR_SIZE << endl;
    return -1;
  }
  else
    {
      double res(0);
      for (int i=0; i<3; i++)
	res += ( (p1[i]-p2[i])*(p1[i]-p2[i]) );
      res = sqrt(res);
      double innerProd(0);
      for (int i=3; i<7; i++)
	innerProd += p1[i]*p2[i];
      res = res * (1 + innerProd*innerProd); // k*innerProd^2 for hole distance
      return res;
    }
}
      

// overload of a term-wise vector-summation method (left+right)
template <typename T>
vector<T> operator+(vector<T> &left, vector<T>& right)
{
    auto len = left.size();
    vector<T> res(len, 0);
    if (left.size() != right.size())
        cout << WARN_VECTOR_SIZE << endl;

    for (int i = 0; i < len; ++i)
    {
        res[i] = left[i] + right[i];
    }
    return res;
}

// overload of a term-wise vector-summation method (left+right)
template <typename T>
void operator+=(vector<T> &left, vector<T>& right)
{
    auto len = left.size();
    if (left.size() != right.size())
        cout << WARN_VECTOR_SIZE << endl;
    for (int i = 0; i < len; ++i)
    {
        left[i] += right[i];
    }

}

// overload of a term-wise vector-division method (left/r)
template <typename T1, typename T2>
vector<double> operator/(vector<T1> &left, T2 r)
{
    auto len = left.size();
    vector<double> res;
    res.resize(len, 0);
    for (int i = 0; i < len; ++i)
    {
        res[i] = left[i] / static_cast<double>(r);
    }
    return res;
}
template <typename T1, typename T2>
vector<double> operator/(vector<T1> &&left, T2 r)
{
    auto len = left.size();
    vector<double> res;
    res.resize(len, 0);
    for (int i = 0; i < len; ++i)
    {
        res[i] = left[i] / static_cast<double>(r);
    }
    return res;
}

// overload of a term-wise vector-division method (left/r)
template <typename T>
void operator/=(vector<T> &left, double r)
{
    auto &&res = left/r;
    swap(res, left);
}

// returns the vector where every term of left is put to the power p
template <typename T>
vector<double> mypow(vector<T> &left, double p)
{
    auto len = left.size();
    vector<double> res;
    res.resize(len);
    for (int i = 0; i < len; ++i)
    {
        res[i] = pow(left[i], p);
    }
    return res;
}
template <typename T>
vector<double> mypow(vector<T> &&left, double p)
{
    auto len = left.size();
    vector<double> res;
    res.resize(len);
    for (int i = 0; i < len; ++i)
    {
        res[i] = pow(left[i], p);
    }
    return res;
}

// overload of a term-wise vector-difference method (left-right)
template <typename T>
vector<T> operator-(vector<T> &left, vector<T> &right)
{
    if (left.size() != right.size())
        cout << WARN_VECTOR_SIZE << endl;
    auto len = left.size();
    vector<T> res;
    res.resize(len);
    for (int i = 0; i < len; ++i) {
        res[i] = left[i] - right[i];
    }
    return res;
}

// overload of a term-wise vector-summation method (left+right)
template <typename T>
void operator+=(vector<T>& left, vector<T>&& right)
{
    if (left.size() != right.size())
        cout << WARN_VECTOR_SIZE << endl;
    auto len(left.size());
    for (int i = 0; i < len; ++i) {
        left[i] += right[i];
    }
}

// returns a vector where every term of left has been square rooted
template <typename T>
vector<double> vector_sqrt(vector<T>& left)
{
    auto len(left.size());
    vector<double> res;
    res.resize(len);
    for (int i = 0; i < len; ++i) {
        res[i] = sqrt(left[i]);
    }
    return res;
}

// returns a vector where every term of left has been multiplied by the scalar d
template<typename T>
vector<double> operator*(vector<double> &left, T d) {
    vector<double> res(left);
    for (auto &item : res) {
        item *= d;
    }
    return res;
}

template<typename T>
vector<T> operator+(vector<T> left, vector<T> right) {
    vector<T> res(left);
    for (int i = 0; i < left.size(); ++i) {
        res[i] += right[i];
    }
    return res;
}

// displays tvec
template<typename T>
ostream &operator<<(ostream &out, vector<T> &tvec) {
    auto len(tvec.size());
    for (int i = 0; i < len; ++i) {
        out << tvec[i];
        if (i < len-1)
            out << ",";
    }
    return out;
}
template<typename T>
ostream &operator<<(ostream &out, const vector<T> &tvec) {
    auto len(tvec.size());
    for (int i = 0; i < len; ++i) {
        out << tvec[i];
        if (i < len-1)
            out << ",";
    }
    return out;
}
#endif //ISODATA_COMMON_H
