/**
 * Tests for various image distance metrics on the Bielefeld image database.
 *
 * @author Andrew Vardy
 */

#include <cmath>
#include "Sim.h"
#include "ImgOps.h"
#include "ImgWindow.h"
#include "SingleConfig.h"
#include "HissAlg.h"
#include "ScaleDiffAlg.h"

int width = 10;
int height = 17;

// Database from which snapshot images will be drawn.
Imgdb ssDB("/data/db/preproc/original", width, height);

// Database from which current images will be drawn.  Also passed to Sim.
Imgdb cvDB("/data/db/preproc/chairs", width, height);

TotalHomingAlg *homingAlg;
int imgWidth = ssDB.getImageWidth();
int imgHeight = ssDB.getImageHeight();

bool allSnapshots = false; // Utilize all images as snapshots?
bool selectedSnapshots = false; // Only utilize 'nSS' snapshot images given
int nSS = 4;                    // below.
int sxArray[4] = {3, 9, 0, 6};
int syArray[4] = {4, 4, 12, 12};
int sx = 7;  // If allSnapshots and selectedSnapshots are both false, utilize
int sy = 8;  // (sx, sy) as the only snapshot.

void similarityMap(int sx, int sy) {
    Img Abs(width, height), Pos(width, height), Neg(width, height);
    Abs.setAll(0);
    Pos.setAll(0);
    Neg.setAll(0);
    Img *SS = ssDB.getImg(sx, sy);
    homingAlg->snapshot(SS);

    for (int cy=0; cy < cvDB.getHeight(); cy++)
        for (int cx=0; cx < cvDB.getWidth(); cx++) {
            if (cx != sx || cy != sy) {
                Img *CV = cvDB.getImg(cx, cy);
                double homeAngle, similarity;
                homingAlg->getHomeAngle(CV, homeAngle, similarity);
                Abs.set(cx, cy, fabs(similarity));
                if (similarity > 0)
                    Pos.set(cx, cy, similarity);
                else
                    Neg.set(cx, cy, -similarity);
            }
        }
    ostringstream absTitle;
    absTitle << "abs_" << sx << "_" << sy << ".png";
    Abs.save(absTitle.str());

    ostringstream posTitle;
    posTitle << "pos_" << sx << "_" << sy << ".png";
    Pos.save(posTitle.str());

    ostringstream negTitle;
    negTitle << "neg_" << sx << "_" << sy << ".png";
    Neg.save(negTitle.str());
}

int main( int argc, char ** argv ) {

//    homingAlg = new HissAlg(imgWidth, imgHeight);
    homingAlg = new ScaleDiffAlg(imgWidth, imgHeight);

    if (allSnapshots) {
        for (int sy=0; sy < ssDB.getHeight(); sy++)
            for (int sx=0; sx < ssDB.getWidth(); sx++)
                similarityMap(sx, sy);

    } else if (selectedSnapshots) {
        for (int i=0; i<nSS; i++)
            similarityMap(sxArray[i], syArray[i]);

    } else {
        similarityMap(sx, sy);
    }

//    ImgWindow::pauseWindow();
    delete homingAlg;
}
