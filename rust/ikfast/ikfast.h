// -*- coding: utf-8 -*-
// Copyright (C) 2012 Rosen Diankov <rosen.diankov@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <vector>
#include <list>
#include <stdexcept>

#ifndef IKFAST_HEADER_COMMON
#define IKFAST_HEADER_COMMON

#define IKFAST_VERSION 62

namespace ikfast {

template <typename T>
class IkSingleDOFSolutionBase
{
public:
IkSingleDOFSolutionBase() : fmul(0), foffset(0), freeind(-1), maxsolutions(1) {
indices[0] = indices[1] = indices[2] = indices[3] = indices[4] = -1;
}
T fmul, foffset;
signed char freeind;
unsigned char jointtype;
unsigned char maxsolutions;
unsigned char indices[5];
};

template <typename T>
class IkSolutionBase
{
public:
virtual ~IkSolutionBase() {
}
virtual void GetSolution(T* solution, const T* freevalues) const = 0;

virtual void GetSolution(std::vector<T>& solution, const std::vector<T>& freevalues) const {
solution.resize(GetDOF());
GetSolution(&solution.at(0), freevalues.size() > 0 ? &freevalues.at(0) : NULL);
}

virtual const std::vector<int>& GetFree() const = 0;

virtual const int GetDOF() const = 0;
};

template <typename T>
class IkSolutionListBase
{
public:
virtual ~IkSolutionListBase() {
}

virtual size_t AddSolution(const std::vector<IkSingleDOFSolutionBase<T> >& vinfos, const std::vector<int>& vfree) = 0;

virtual const IkSolutionBase<T>& GetSolution(size_t index) const = 0;

virtual size_t GetNumSolutions() const = 0;

virtual void Clear() = 0;
};

template <typename T>
class IkFastFunctions
{
public:
IkFastFunctions() : _ComputeIk(NULL), _ComputeFk(NULL), _GetNumFreeParameters(NULL), _GetFreeParameters(NULL), _GetNumJoints(NULL), _GetIkRealSize(NULL), _GetIkFastVersion(NULL), _GetIkType(NULL), _GetKinematicsHash(NULL) {
}
virtual ~IkFastFunctions() {
}
typedef bool (*ComputeIkFn)(const T*, const T*, const T*, IkSolutionListBase<T>&);
ComputeIkFn _ComputeIk;
typedef void (*ComputeFkFn)(const T*, T*, T*);
ComputeFkFn _ComputeFk;
typedef int (*GetNumFreeParametersFn)();
GetNumFreeParametersFn _GetNumFreeParameters;
typedef int* (*GetFreeParametersFn)();
GetFreeParametersFn _GetFreeParameters;
typedef int (*GetNumJointsFn)();
GetNumJointsFn _GetNumJoints;
typedef int (*GetIkRealSizeFn)();
GetIkRealSizeFn _GetIkRealSize;
typedef const char* (*GetIkFastVersionFn)();
GetIkFastVersionFn _GetIkFastVersion;
typedef int (*GetIkTypeFn)();
GetIkTypeFn _GetIkType;
typedef const char* (*GetKinematicsHashFn)();
GetKinematicsHashFn _GetKinematicsHash;
};

// Implementations of the abstract classes, user doesn't need to use them

template <typename T>
class IkSolution : public IkSolutionBase<T>
{
public:
IkSolution(const std::vector<IkSingleDOFSolutionBase<T> >& vinfos, const std::vector<int>& vfree) {
_vbasesol = vinfos;
_vfree = vfree;
}

virtual void GetSolution(T* solution, const T* freevalues) const {
for(std::size_t i = 0; i < _vbasesol.size(); ++i) {
if( _vbasesol[i].freeind < 0 )
solution[i] = _vbasesol[i].foffset;
else {
solution[i] = freevalues[_vbasesol[i].freeind]*_vbasesol[i].fmul + _vbasesol[i].foffset;
if( solution[i] > T(3.14159265358979) ) {
solution[i] -= T(6.28318530717959);
}
else if( solution[i] < T(-3.14159265358979) ) {
solution[i] += T(6.28318530717959);
}
}
}
}

virtual void GetSolution(std::vector<T>& solution, const std::vector<T>& freevalues) const {
solution.resize(GetDOF());
GetSolution(&solution.at(0), freevalues.size() > 0 ? &freevalues.at(0) : NULL);
}

virtual const std::vector<int>& GetFree() const {
return _vfree;
}
virtual const int GetDOF() const {
return static_cast<int>(_vbasesol.size());
}

virtual void Validate() const {
for(size_t i = 0; i < _vbasesol.size(); ++i) {
if( _vbasesol[i].maxsolutions == (unsigned char)-1) {
throw std::runtime_error("max solutions for joint not initialized");
}
if( _vbasesol[i].maxsolutions > 0 ) {
if( _vbasesol[i].indices[0] >= _vbasesol[i].maxsolutions ) {
throw std::runtime_error("index >= max solutions for joint");
}
if( _vbasesol[i].indices[1] != (unsigned char)-1 && _vbasesol[i].indices[1] >= _vbasesol[i].maxsolutions ) {
throw std::runtime_error("2nd index >= max solutions for joint");
}
}
}
}

virtual void GetSolutionIndices(std::vector<unsigned int>& v) const {
v.resize(0);
v.push_back(0);
for(int i = (int)_vbasesol.size()-1; i >= 0; --i) {
if( _vbasesol[i].maxsolutions != (unsigned char)-1 && _vbasesol[i].maxsolutions > 1 ) {
for(size_t j = 0; j < v.size(); ++j) {
v[j] *= _vbasesol[i].maxsolutions;
}
size_t orgsize=v.size();
if( _vbasesol[i].indices[1] != (unsigned char)-1 ) {
for(size_t j = 0; j < orgsize; ++j) {
v.push_back(v[j]+_vbasesol[i].indices[1]);
}
}
if( _vbasesol[i].indices[0] != (unsigned char)-1 ) {
for(size_t j = 0; j < orgsize; ++j) {
v[j] += _vbasesol[i].indices[0];
}
}
}
}
}

std::vector< IkSingleDOFSolutionBase<T> > _vbasesol;
std::vector<int> _vfree;
};

template <typename T>
class IkSolutionList : public IkSolutionListBase<T>
{
public:
virtual size_t AddSolution(const std::vector<IkSingleDOFSolutionBase<T> >& vinfos, const std::vector<int>& vfree)
{
size_t index = _listsolutions.size();
_listsolutions.push_back(IkSolution<T>(vinfos,vfree));
return index;
}

virtual const IkSolutionBase<T>& GetSolution(size_t index) const
{
if( index >= _listsolutions.size() ) {
throw std::runtime_error("GetSolution index is invalid");
}
typename std::list< IkSolution<T> >::const_iterator it = _listsolutions.begin();
std::advance(it,index);
return *it;
}

virtual size_t GetNumSolutions() const {
return _listsolutions.size();
}

virtual void Clear() {
_listsolutions.clear();
}

protected:
std::list< IkSolution<T> > _listsolutions;
};

}

#endif // OPENRAVE_IKFAST_HEADER

// The following code is dependent on the C++ library linking with.
#ifdef IKFAST_HAS_LIBRARY

// defined when creating a shared object/dll
#ifdef IKFAST_CLIBRARY
#ifdef _MSC_VER
#define IKFAST_API extern "C" __declspec(dllexport)
#else
#define IKFAST_API extern "C"
#endif
#else
#define IKFAST_API
#endif

#ifdef IKFAST_NAMESPACE
namespace IKFAST_NAMESPACE {
#endif

#ifdef IKFAST_REAL
typedef IKFAST_REAL IkReal;
#else
typedef double IkReal;
#endif

IKFAST_API bool ComputeIk(const IkReal* eetrans, const IkReal* eerot, const IkReal* pfree, ikfast::IkSolutionListBase<IkReal>& solutions);

IKFAST_API void ComputeFk(const IkReal* joints, IkReal* eetrans, IkReal* eerot);

IKFAST_API int GetNumFreeParameters();

IKFAST_API int* GetFreeParameters();

IKFAST_API int GetNumJoints();

IKFAST_API int GetIkRealSize();

IKFAST_API const char* GetIkFastVersion();

IKFAST_API int GetIkType();

IKFAST_API const char* GetKinematicsHash();

#ifdef IKFAST_NAMESPACE
}
#endif

#endif // IKFAST_HAS_LIBRARY
