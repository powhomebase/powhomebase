/**********************************************************************************************************************/
/* Description: Header containing a bunch of units convertors                                                         */
/**********************************************************************************************************************/
#pragma once

/**********************************************************************************************************************/
/* Macros                                                                                                             */
/**********************************************************************************************************************/

/* Time units */

#define MS_IN_S 1000ULL
#define US_IN_S 1000000ULL
#define NS_IN_S 1000000000ULL

/* Warning - not double - will round down! */
#define PS_TO_NS(ps_)   ((ps_) / 1000U)
#define PS_TO_US(ps_)   NS_TO_US(PS_TO_NS(ps_))
#define PS_TO_MS(ps_)   US_TO_MS(PS_TO_US(ps_))
#define PS_TO_S(ps_)    MS_TO_S(PS_TO_MS(ps_))
#define PS_TO_MIN(ps_)  S_TO_MIN(PS_TO_S(ps_))
#define PS_TO_HOUR(ps_) MIN_TO_HOUR(PS_TO_MIN(ps_))
#define PS_TO_DAY(ps_)  HOUR_TO_DAY(PS_TO_HOUR(ps_))

#define NS_TO_PS(ns_)   ((ns_)*1000ULL)
#define NS_TO_US(ns_)   ((ns_) / 1000U)
#define NS_TO_MS(ns_)   US_TO_MS(NS_TO_US(ns_))
#define NS_TO_S(ns_)    MS_TO_S(NS_TO_MS(ns_))
#define NS_TO_MIN(ns_)  S_TO_MIN(NS_TO_S(ns_))
#define NS_TO_HOUR(ns_) MIN_TO_HOUR(NS_TO_MIN(ns_))
#define NS_TO_DAY(ns_)  HOUR_TO_DAY(NS_TO_HOUR(ns_))

#define US_TO_PS(us_)   NS_TO_PS(US_TO_NS(us_))
#define US_TO_NS(us_)   ((us_)*1000ULL)
#define US_TO_MS(us_)   ((us_) / 1000U)
#define US_TO_S(us_)    MS_TO_S(US_TO_MS(us_))
#define US_TO_MIN(us_)  S_TO_MIN(US_TO_S(us_))
#define US_TO_HOUR(us_) MIN_TO_HOUR(US_TO_MIN(us_))
#define US_TO_DAY(us_)  HOUR_TO_DAY(US_TO_HOUR(us_))

#define MS_TO_PS(ms_)   NS_TO_PS(MS_TO_NS(ms_))
#define MS_TO_NS(ms_)   US_TO_NS(MS_TO_US(ms_))
#define MS_TO_US(ms_)   ((ms_)*1000ULL)
#define MS_TO_S(ms_)    ((ms_) / 1000U)
#define MS_TO_MIN(ms_)  S_TO_MIN(MS_TO_S(ms_))
#define MS_TO_HOUR(ms_) MIN_TO_HOUR(MS_TO_MIN(ms_))
#define MS_TO_DAY(ms_)  HOUR_TO_DAY(MS_TO_HOUR(ms_))

#define S_TO_PS(s_)   NS_TO_PS(S_TO_NS(s_))
#define S_TO_NS(s_)   US_TO_NS(S_TO_US(s_))
#define S_TO_US(s_)   MS_TO_US(S_TO_MS(s_))
#define S_TO_MS(s_)   ((s_)*1000ULL)
#define S_TO_MIN(s_)  ((s_) / 60U)
#define S_TO_HOUR(s_) MIN_TO_HOUR(S_TO_MIN(s_))
#define S_TO_DAY(s_)  HOUR_TO_DAY(S_TO_HOUR(s_))

#define MIN_TO_PS(m_)   NS_TO_PS(MIN_TO_NS(m_))
#define MIN_TO_NS(m_)   US_TO_NS(MIN_TO_US(m_))
#define MIN_TO_US(m_)   MS_TO_US(MIN_TO_MS(m_))
#define MIN_TO_MS(m_)   S_TO_MS(MIN_TO_S(m_))
#define MIN_TO_S(m_)    ((m_)*60ULL)
#define MIN_TO_HOUR(m_) ((m_) / 60U)
#define MIN_TO_DAY(m_)  HOUR_TO_DAY(MINS_TO_HOUR(m_))

#define HOUR_TO_PS(h_)  NS_TO_PS(HOUR_TO_NS(h_))
#define HOUR_TO_NS(h_)  US_TO_NS(HOUR_TO_US(h_))
#define HOUR_TO_US(h_)  MS_TO_US(HOUR_TO_MS(h_))
#define HOUR_TO_MS(h_)  S_TO_MS(HOUR_TO_S(h_))
#define HOUR_TO_S(h_)   MIN_TO_S(HOUR_TO_MIN(h_))
#define HOUR_TO_MIN(h_) ((h_)*60ULL)
#define HOUR_TO_DAY(h_) ((h_) / 24U)

#define DAY_TO_PS(d_)   NS_TO_PS(DAY_TO_NS(d_))
#define DAY_TO_NS(d_)   US_TO_NS(DAY_TO_US(d_))
#define DAY_TO_US(d_)   MS_TO_US(DAY_TO_MS(d_))
#define DAY_TO_MS(d_)   S_TO_MS(DAY_TO_S(d_))
#define DAY_TO_S(d_)    MIN_TO_S(DAY_TO_MIN(d_))
#define DAY_TO_MIN(d_)  HOUR_TO_MIN(DAY_TO_HOUR(d_))
#define DAY_TO_HOUR(d_) ((d_)*24ULL)
