#ifndef TIC_TOC_H
#define TIC_TOC_H

#include <ctime>
#include <cstdlib>

class TicToc
{
public:
    TicToc(): sum(0)
    {
        tic();
    }

    void tic()
    {
        t = clock();
    }

    double toc()
    {
        sum += (clock() - t) / CLOCKS_PER_SEC * 1000;
        return sum;
    }
private:
    double sum, t;
};

#endif
