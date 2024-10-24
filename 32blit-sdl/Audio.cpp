#include <algorithm>
#include <cstdio>
#include <iostream>
#include "SDL.h"

#include "Audio.hpp"
#include "audio/audio.hpp"
#include "engine/api_private.hpp"

static void _audio_callback(void *userdata, uint8_t *stream, int len);

Audio::Audio() {
    SDL_AudioSpec desired = {}, audio_spec = {};

    desired.freq = _sample_rate;
    desired.format = AUDIO_S16LSB;
    desired.channels = 1;

    desired.samples = 256;
    desired.callback = _audio_callback;

    audio_device = SDL_OpenAudioDevice(nullptr, 0, &desired, &audio_spec, 0);

    if(audio_device == 0){
        std::cerr << "Audio Init Failed: " << SDL_GetError() << std::endl;
    }

    SDL_PauseAudioDevice(audio_device, 0);
}

Audio::~Audio() {
    SDL_PauseAudioDevice(audio_device, 1);
    SDL_CloseAudioDevice(audio_device);
}

static void _audio_bufferfill(short *buffer, int buffer_size){
    memset(buffer, 0, buffer_size);

    for(auto sample = 0; sample < buffer_size; sample++){
        buffer[sample] = (int)blit::get_audio_frame() - 0x8000;
    }
}

static void _audio_callback(void *userdata, uint8_t *stream, int len){
    _audio_bufferfill((short *)stream, len / 2);
}
