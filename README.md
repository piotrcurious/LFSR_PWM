# LFSR_PWM
LFSR based PWM with brute force finders

LFSRs through the linked-list lens (and why the “random” sequence is number theory)

Short version up front: an LFSR is just a tiny self-referencing linked list of bits whose “next” value is computed by XOR’ing (i.e. adding in GF(2)) some earlier nodes. Viewed algebraically, every shift = multiply by  mod a polynomial over GF(2). If that polynomial is primitive, the shifts cycle through all nonzero -bit patterns (period ). So the illusion of randomness is a deterministic consequence of linear algebra and number theory in finite fields.


---

1) The linked-list picture (intuitive)

Imagine a singly linked list of  nodes arranged in a row: node0 → node1 → ... → node(n-1). Each node holds a bit (0 or 1). A single LFSR clock does two things:

1. compute a new bit new_head by XOR’ing the bits stored in a small set of tap nodes (e.g. node0 and node2),


2. drop the tail node and insert new_head at the head — every node shifts one position to the right.



Pseudocode for one tick (Fibonacci form):

new_head = tap1.value XOR tap2.value XOR ...
drop tail (node[n-1])
for i from n-1 down to 1:
    node[i].value = node[i-1].value
node[0].value = new_head

That is literally a linked list whose content at the head is self-referencing: the new head is computed from values already in the list. The “list” keeps evolving deterministically by reusing (referencing) its own previous values.


---

2) Two hardware flavours: Fibonacci vs Galois (still lists, just different wiring)

Fibonacci LFSR (what I described): compute new_head = XOR(taps) and shift. The taps are read to form the incoming bit. That’s like computing the new head from several existing nodes.

Galois LFSR: instead of computing one new_head, the output bit is shifted and, if the output bit is 1, you XOR that output into certain tapped nodes during the shift. Mechanically it updates several nodes in place as the shift happens. This is equivalent mathematically, but the “wiring” is different — like a linked list where, when you pop the head, you conditionally flip some nodes in the list during the move. Galois form often maps directly to cheaper hardware because you avoid computing a multi-input XOR for every clock.


Both forms are linear; they are just different linear maps on the same -bit state.


---

3) Linear algebra view (compact, exact)

Treat the  bits as a column vector . Each clock is a linear map:

\mathbf{s}_{t+1} = C\,\mathbf{s}_t

where  is the companion matrix determined by the tap polynomial

p(x) = x^n + c_{n-1}x^{n-1} + \dots + c_1 x + c_0\quad (c_i\in\{0,1\}).

The recurrence the LFSR implements is the linear recurrence whose characteristic polynomial is . All future states are powers of  applied to the initial state: . So the full sequence is completely determined by  (i.e. the polynomial) and the seed .


---

4) Where Galois fields and number theory enter

Map the -bit vector to a polynomial  over GF(2):

\mathbf{s} = [a_{n-1},\dots,a_1,a_0] \;\longleftrightarrow\; a(x)=a_{n-1}x^{n-1}+\dots+a_1 x + a_0.

One clock of the LFSR (Fibonacci form) corresponds exactly to multiplying  by  and reducing modulo :

a_{t+1}(x) \equiv x \, a_t(x) \pmod{p(x)}.

Thus the sequence of states is

a_0(x),\; x\,a_0(x) \bmod p,\; x^2 a_0(x) \bmod p,\; \dots

Key number-theory fact: if  is primitive (a special irreducible polynomial), then the residue class of  is a generator of the multiplicative group , whose order is . Multiplying repeatedly by a generator cycles through all nonzero elements. Therefore the LFSR will have maximal period  — an m-sequence.

So the “magic” is: powers of one element in a finite field behave like a long pseudorandom cycle. The apparent randomness is structured: it is the orbit of multiplication by  in a finite cyclic group.


---

5) Example: , polynomial 

This polynomial is primitive. Start with seed (1,0,1) which represents .

Successive states (showing the 3-bit vector after each clock) cycle through 7 nonzero states:

seed: 101  (= x^2 + 1)
next: 010
next: 001
next: 100
next: 110
next: 111
next: 011
then back to 101 ...

Period = . The sequence looks random-ish, but it’s exactly the sequence of powers  mod .


---

6) Why LFSR output looks random but is not cryptographically random

Good reasons it appears random:

m-sequences have balanced numbers of 1s and 0s (for long runs), predictable autocorrelation properties, and well-distributed run lengths — consequences of algebraic structure.

Linear structure gives very long periods (up to ) while being extremely cheap to implement.


Why it’s not secure by itself:

The mapping is linear over GF(2). Given enough output, linear algebra (Berlekamp–Massey) reconstructs the minimal polynomial and the state, so the generator can be predicted. That’s why stream ciphers add nonlinearity or combinations of multiple LFSRs.



---

7) Galois interpretation of “self-reference”

Calling an LFSR a “self-referencing linked list” is accurate in two senses:

operational: the new head bit is computed from (referenced) bits already in the list, so the list references itself to compute its next state;

algebraic: the state at time  is the linear combination of previous state bits — the state vector is being multiplied by a matrix built from the polynomial that references earlier coordinates.


In Galois form the “self-reference” is even more local: when a node’s bit leaves the list it is XOR’d into certain other nodes — like the departing node waving its value into particular nodes it points to.


---

8) Small implementation sketch (Fibonacci LFSR; readable)

# n-bit Fibonacci LFSR, taps is list of indices (0 is left/head)
def step(state, taps):
    # state: list of n bits, head is state[0]
    new_head = 0
    for t in taps:
        new_head ^= state[t]
    # shift right
    state = [new_head] + state[:-1]
    return state

# example
state = [1,0,1]   # 3-bit seed
taps = [0,2]      # corresponds to x^3 + x + 1 (taps at head and last-before-tail)
for _ in range(7):
    print(state)
    state = step(state, taps)


---

Takeaway (one-line)

An LFSR is a tiny deterministic linked list whose self-referencing XOR wiring implements multiplication by  in the finite field ; choose a primitive polynomial and that multiplication becomes a generator of the field’s multiplicative group, producing a long, well-behaved pseudorandom cycle — all of which is pure linear algebra and elementary number theory, not magic.
