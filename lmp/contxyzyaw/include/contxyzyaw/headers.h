#ifndef __HEADERS1_H_
#define __HEADERS1_H_

#include <sbpl/sbpl_exception.h>
#include <sbpl/config.h>

#if MEM_CHECK == 1
#define _CRTDBG_MAP_ALLOC
#define CRTDBG_MAP_ALLOC
#endif

#include <stdlib.h>
#if MEM_CHECK == 1
#include <crtdbg.h>
#endif


// include necessary environment file
#include <sbpl/discrete_space_information/environment.h>

#include <sbpl/heuristics/heuristic.h>
#include <sbpl/heuristics/embedded_heuristic.h>

#include <sbpl/planners/araplanner.h>
#include <sbpl/planners/planner.h>
#include <sbpl/planners/mhaplanner.h>

#include <sbpl/utils/2Dgridsearch.h>
#include <sbpl/utils/heap.h>
#include <sbpl/utils/list.h>
#include <sbpl/utils/key.h>
#include <sbpl/utils/mdp.h>
#include <sbpl/utils/mdpconfig.h>
#include <sbpl/utils/sbpl_fifo.h>
#include <sbpl/utils/sbpl_bfs_2d.h>
#include <sbpl/utils/sbpl_bfs_3d.h>
#include <sbpl/utils/utils.h>

#endif
