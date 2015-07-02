

// System includes
#include <iostream>
#include <climits>

#include "lap.h"

#include <stdlib.h>


int main(int argc, char **argv)
{
    typedef int row;
    typedef int col;
    typedef double cost;

    col* rowsol;
    row* colsol;
    cost *u;
    cost *v;
    double lapcosts;
    double **ccosts;



    int n = 25;

    ccosts = (cost **)malloc(sizeof(cost *)*n);

    for(size_t i = 0; i<n; i++){
     ccosts[i] = (cost *)malloc(sizeof(cost)*n);
    }

    int s;
    for(size_t i = 0; i<n; i++){
      for(size_t j = 0; j<n; j++){
        ccosts[i][j] = s;
        s++;
        std::cout << s << " ";
      }
      std::cout << std::endl;
    }

    rowsol = (col *)malloc(sizeof(col)*n);
    colsol = (row *)malloc(sizeof(row)*n);
    u = (cost *)malloc(sizeof(cost)*n);
    v = (cost *)malloc(sizeof(cost)*n);

    lapcosts = lap(n,ccosts,rowsol,colsol,u,v);

    std::cout << "rowsol: ";
    for (int i = 0; i < n; i++){
       std::cout << rowsol[i] << " ";
    }
    std::cout << std::endl;

    std::cout << "colsol:";
    for (int i = 0; i < n; i++){
      std::cout << colsol[i] << " ";
    }
    std::cout << std::endl;

    std::cout << "costs:" << lapcosts << std::endl;


    for (int i = 0; i < n; i++){
       free(ccosts[i]);
    }


    free(ccosts);
    free(rowsol);
    free(colsol);
    free(u);
    free(v);

}
