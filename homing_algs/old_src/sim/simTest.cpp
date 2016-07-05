#include <unistd.h>
#include "Sim.h"
#include "rng.h"

int main( int argc, char ** argv ) {
    Imgdb db("/data/db/preproc/original", 10, 17);
    Sim sim("TESTING SIM", &db);
    RNG random(0);

    sim.teleport(sim.getWidth()/2, 0, M_PI/2.0, true);
    sim.getSS();
    for ( int i=0; i<16; i++ ) {
        sim.move( random.uniform(-1.57, 1.57), 0.5);
        sim.move(1, 0);
        sim.getCV();
    }

    Img::printNumberOfImages();
    ImgWindow::waitAll();
}
