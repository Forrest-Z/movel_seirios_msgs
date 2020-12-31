////////////////////////////////////////////////////////////////////
// Copyright (C) 2010, SafeNet, Inc. All rights reserved.
//
// HASP(R) is a registered trademark of SafeNet, Inc.
//
//
//
////////////////////////////////////////////////////////////////////

#include <movel_hasp_vendor/hasp_api_cpp_.h>

////////////////////////////////////////////////////////////////////
//! \struct ChaspLock haspcpp_.h
//! \brief Class providing the thread locking interface
//! (platform dependent).
////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////
// Windows
////////////////////////////////////////////////////////////////////
#if defined(HASP_HL_TARGET_WINDOWS)

////////////////////////////////////////////////////////////////////
// Construction/Destruction
////////////////////////////////////////////////////////////////////

ChaspLock::ChaspLock() : m_bInit(false)
{
  __try
  {
    ::InitializeCriticalSection(&m_critLock);
    m_bInit = true;
  }
  __except (EXCEPTION_EXECUTE_HANDLER)
  {
  }
}

ChaspLock::~ChaspLock()
{
  if (m_bInit)
  {
    ::DeleteCriticalSection(&m_critLock);
    m_bInit = false;
  }
}

////////////////////////////////////////////////////////////////////
// Implementation
////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////
//
////////////////////////////////////////////////////////////////////
bool ChaspLock::isInit() const
{
  return m_bInit;
}

////////////////////////////////////////////////////////////////////
//
////////////////////////////////////////////////////////////////////
bool ChaspLock::lock()
{
  DIAG_ASSERT(m_bInit);

  if (!m_bInit)
    return false;

  __try
  {
    ::EnterCriticalSection(&m_critLock);
    return true;
  }
  __except (EXCEPTION_EXECUTE_HANDLER)
  {
  }

  return false;
}

////////////////////////////////////////////////////////////////////
//
////////////////////////////////////////////////////////////////////
bool ChaspLock::unlock()
{
  DIAG_ASSERT(m_bInit);

  if (!m_bInit)
    return false;

  __try
  {
    ::LeaveCriticalSection(&m_critLock);
    return true;
  }
  __except (EXCEPTION_EXECUTE_HANDLER)
  {
  }

  return false;
}

#else

////////////////////////////////////////////////////////////////////
// Construction/Destruction
////////////////////////////////////////////////////////////////////

ChaspLock::ChaspLock() : m_bInit(false)
{
  m_bInit = (0 == ::pthread_mutexattr_init(&m_attrRecursive)) &&
            (0 == ::pthread_mutexattr_settype(&m_attrRecursive, PTHREAD_MUTEX_RECURSIVE)) &&
            (0 == ::pthread_mutex_init(&m_mutex, &m_attrRecursive));
}

ChaspLock::~ChaspLock()
{
  if (m_bInit)
  {
    pthread_mutex_destroy(&m_mutex);
    pthread_mutexattr_destroy(&m_attrRecursive);
  }
}

////////////////////////////////////////////////////////////////////
// Implementation
////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////
//
////////////////////////////////////////////////////////////////////
bool ChaspLock::isInit() const
{
  return m_bInit;
}

////////////////////////////////////////////////////////////////////
//
////////////////////////////////////////////////////////////////////
bool ChaspLock::lock()
{
  DIAG_ASSERT(m_bInit);
  return m_bInit && (0 == ::pthread_mutex_lock(&m_mutex));
}

////////////////////////////////////////////////////////////////////
//
////////////////////////////////////////////////////////////////////
bool ChaspLock::unlock()
{
  DIAG_ASSERT(m_bInit);
  return m_bInit && (0 == ::pthread_mutex_unlock(&m_mutex));
}

#endif  // HASP_HL_TARGET_WINDOWS
