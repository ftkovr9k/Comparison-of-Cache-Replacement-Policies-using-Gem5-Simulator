/*
 * Copyright (c) 2004-2006 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "cpu/pred/gshare.hh"

#include "base/intmath.hh"
#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/Fetch.hh"

namespace gem5
{

namespace branch_prediction
{

GshareBP::GshareBP(const GshareBPParams &params)
    : BPredUnit(params),
      localPredictorSize(params.localPredictorSize),
      localPredictorSizeB(params.localPredictorSizeB),
      localCtrBits(params.localCtrBits),
      localCtrBitsB(params.localCtrBitsB),
      localPredictorSets(localPredictorSize / localCtrBits),
      localCtrs(localPredictorSets, SatCounter8(localCtrBits)),
      indexMask(localPredictorSets - 1),
      bhsrt(pow(2,8), (uint8_t)0),
      bhsrtMask(params.bhsrtMask)
{
    if (!isPowerOf2(localPredictorSize)) {
        fatal("Invalid local predictor size!\n");
    }

    if (!isPowerOf2(localPredictorSets)) {
        fatal("Invalid number of local predictor sets! Check localCtrBits.\n");
    }

    DPRINTF(Fetch, "index mask: %#x\n", indexMask);

    DPRINTF(Fetch, "local predictor size: %i\n",
            localPredictorSize);

    DPRINTF(Fetch, "local counter bits: %i\n", localCtrBits);

    DPRINTF(Fetch, "instruction shift amount: %i\n",
            instShiftAmt);
}

void
GshareBP::btbUpdate(ThreadID tid, Addr branch_addr, void * &bp_history)
{
// Place holder for a function that is called to update predictor history when
// a BTB entry is invalid or not found.
}


bool
GshareBP::lookup(ThreadID tid, Addr branch_addr, void * &bp_history)
{
    bool taken;
    unsigned bht_idx = getLocalIndex(branch_addr);
    unsigned bhsrt_predictor_idx = getBhsrtIndex(branch_addr);
    unsigned bhsrt_extended = bhsrt[bhsrt_predictor_idx] &indexMask;
    unsigned xor_ed = bhsrt_extended ^ bht_idx; 

    DPRINTF(Fetch, "Looking up index %#x\n",
            bhsrt_predictor_idx);

    uint8_t counter_val = localCtrs[xor_ed];

    DPRINTF(Fetch, "prediction is %i.\n",
            (int)counter_val);

    taken = getPrediction(counter_val);

    return taken;
}

void
GshareBP::update(ThreadID tid, Addr branch_addr, bool taken, void *bp_history,
                bool squashed, const StaticInstPtr & inst, Addr corrTarget)
{
    unsigned bht_idx = getLocalIndex(branch_addr);
    unsigned bhsrt_predictor_idx = getBhsrtIndex(branch_addr);
    unsigned bhsrt_extended = bhsrt[bhsrt_predictor_idx] &indexMask;
    unsigned xor_ed = bhsrt_extended ^ bht_idx;
    assert(bp_history == NULL);
    unsigned local_predictor_idx;

    // No state to restore, and we do not update on the wrong
    // path.
    if (squashed) {
        return;
    }

    // Update the local predictor.
    local_predictor_idx = getBhsrtIndex(branch_addr);

    DPRINTF(Fetch, "Looking up index %#x\n", local_predictor_idx);

    if (taken) {
        DPRINTF(Fetch, "Branch updated as taken.\n");
        localCtrs[xor_ed]++;
	bhsrt[local_predictor_idx] = (bhsrt[local_predictor_idx] << 1)|1;
    } else {
        DPRINTF(Fetch, "Branch updated as not taken.\n");
        localCtrs[xor_ed]--;
	bhsrt[local_predictor_idx] = (bhsrt[local_predictor_idx] << 1);
    }
}

inline
bool
GshareBP::getPrediction(uint8_t &count)
{
    // Get the MSB of the count
    return (count >> (localCtrBits - 1));
}

inline
unsigned
GshareBP::getLocalIndex(Addr &branch_addr)
{
    return (branch_addr >> instShiftAmt) & indexMask;
}

unsigned
GshareBP::getBhsrtIndex(Addr &branch_addr){
  return (branch_addr >> instShiftAmt) & bhsrtMask;
}

void
GshareBP::uncondBranch(ThreadID tid, Addr pc, void *&bp_history)
{
}

} // namespace branch_prediction
} // namespace gem5


