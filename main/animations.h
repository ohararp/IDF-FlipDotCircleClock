#pragma once

// Demo animation: home → sweep hours 1–12 on flipdots (1.5s each) → blank → restore time
void anim_demo(void);

// Chaos animation: 12 random hour positions with motor + flipdot (1.5s each) → restore time
void anim_chaos(void);

// Sync animation: home → synchronized hand sweep to each hour + matching flipdot → restore time
void anim_sync(void);
