#include "sequence_model.h"

void SequenceModel_Reset(SequenceModel* model)
{
    model->initialized = 0;
    model->last_sequence = 0;
    model->last_payload_hash = 0;
}

SequenceDecision SequenceModel_Check(const SequenceModel* model, uint16_t sequence, uint64_t payload_hash)
{
    if (!model->initialized)
        return SEQUENCE_ACCEPT_NEW;
    if (sequence == model->last_sequence)
        return payload_hash == model->last_payload_hash
            ? SEQUENCE_ACCEPT_DUPLICATE : SEQUENCE_REJECT_MISMATCH;
    if (sequence == (uint16_t)(model->last_sequence + 1))
        return SEQUENCE_ACCEPT_NEW;
    return SEQUENCE_REJECT_MISMATCH;
}

void SequenceModel_Commit(SequenceModel* model, uint16_t sequence, uint64_t payload_hash)
{
    model->initialized = 1;
    model->last_sequence = sequence;
    model->last_payload_hash = payload_hash;
}
