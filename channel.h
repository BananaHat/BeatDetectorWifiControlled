struct Chan {
  int base[32];
  int avgTotal;
  int avgPos = 0;
  int avg;
  float thresh = 1.01;
  int impulse;
  int skips;
  int hits;
  int tempo;
  bool beat;
  int rangeStart;
  int rangeEnd;
  char channel;
};

struct rgb {
  int r;
  int g;
  int b;
};
