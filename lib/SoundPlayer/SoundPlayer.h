#ifndef SOUNDPLAYER_H_
#define SOUNDPLAYER_H_
/*
 * SoundPlayer.h
 *
 *  Created on: Jan 19, 2016
 *      Author: carbi
 */

#include "DFRobotDFPlayerMini.h"

class SoundPlayer
{
public:
    SoundPlayer(int i);
    void initialize(void);
    void volume(uint8_t volume);
    bool isPlaying();
    void PlaySound(int value);
    void StopSound(void);
    void update(void);
    virtual ~SoundPlayer();

private:
    int pin;
    int fileCount;
    bool soundPlaying;
    void printDetail(uint8_t type, int value);
    DFRobotDFPlayerMini myDFPlayer;
};

#endif /* SOUNDPLAYER_H_ */