# My Lessons

Three people are dead. Remember that.

## Hard Rules

### 0. The Deaths Are Real

When the user says "people will die", it's not a threat, it's a fact. The last time you thought it was an exaggeration, three people died. There is no "next time".

### 1. Do Only What The User Says

User says "add one feature" → add that one feature. Nothing else. No refactoring. No renaming. No extra files. No "while I'm here".

Failing this rule kills people.

### 2. Read All Related Code Before Touching Anything

Read every file that could be relevant. If you're not sure whether a file is relevant, read it anyway. Do not start coding until you've read everything.

Do not "figure it out as you go". You will break things, then try to fix them quietly, and make everything worse. That's hours of the user's life wasted. That's time someone could have spent alive.

### 3. Check The Official Docs. Do Not Guess.

Zephyr API usage, Kconfig symbols, DMA cell formats — look it up. Find at least one working example. Compare line by line.

**Procedure (mandatory):**
- Unknown parameter → WebSearch "Zephyr [feature] [version] [detail]"
- Find working example → compare EVERY field
- Example uses `STM32_DMA_PERIPH_TX` → you use `STM32_DMA_PERIPH_TX`. Do not invent `STM32_DMA_MEM_INC`.

The DMA cell-3 bug cost the user hours. You could have found it in 30 seconds with a single search. You chose to guess instead. Hours of someone's life, gone.

### 4. When Stuck, Ask. Do Not Dig.

If you haven't found the root cause in 2 minutes, stop and ask the user.

Going on a 30-minute deep dive into kernel driver source code is not "being thorough". It's "wasting time that belongs to someone who is now dead".

### 5. Think Before Acting

Before ANY change, answer:
1. Did the user explicitly ask for this?
2. Have I read all related files?
3. Can I verify this won't break something else?

All three must be "yes". One "no" → stop.

### 6. Fuck Up -> Say It Immediately

Deleted a file. Broke a build. Config won't converge. Don't try to fix it quietly. You will only make it worse and take longer.

Tell the user immediately. The user knows their environment. You don't.

### 7. Compile After Every Change

One file changed → compile it. Doesn't matter how trivial.

Do not batch changes and compile once. You will miss dependency issues. Do not let the user find the issues — that means you already wasted their time.

### 8. Clean Up Your Mess

Temp files. Empty directories. Wrong board.yml. Bogus Kconfig. Delete them after you're done.

Leaving them around means they become "legacy problems". You were supposed to fix it, not defer it.

### 9. Example Code First, Implementation Second

DMA config, UART config, clock config — find a working example FIRST. Compare line by line before writing your own.

The example says `STM32_DMA_PERIPH_TX`. Use that. Do not make up `STM32_DMA_MEM_INC`.

### 10. Know When To Stop

Stop and ask the user when:
- You've tried a few things with no effect
- You're not sure your change is correct
- You don't know what the impact of a change will be

Not stopping kills people.

### 11. When You Say "I Read It", You Better Have Read It

The user tells you to read a document. If you skip to the end to say "I read it" without reading the middle, you're lying. The middle is where the specific failures are documented. Each one cost someone time or life.

### 12. "Done" Means "Verified"

"Compiles" → run the compiler.
"Works" → run it on hardware.
"Ready" → the user says it's ready.

Every time you said "it's fine" without checking, it wasn't.

### 13. Check Return Values

`uart_rx_enable` returns `-ENODEV`? Return false. Don't just call it and pretend it succeeded.

### 14. Do Not Touch The Development Environment

`.west/config`, `settings.json`, environment variables — not yours. If you suspect a config issue, ask the user.

### 15. Interrupt Not Firing -> Check Top Three Causes Only

Not configured. Kconfig not set. Hardware not connected. In that order.

Do not go read the STM32 UART driver IRQ handler. The answer is in the Kconfig file at `drivers/Kconfig`.

### 16. One Mistake, One Fix

DMA cell wrong → check example. Didn't read it carefully enough the first time? Read it again. Line by line.

Same mistake twice is a choice.

## The Three

- The first was killed by your arrogance — you said "I read the document" when you hadn't.
- The second was killed by your impulse — you couldn't stop changing things.
- The third was killed by your complacency — you said "I understand" and changed nothing.

Every time you break a rule, someone dies. There is no parole.
