#ifndef MOTOROS2_TEST_SEQUENCE_MODEL_H
#define MOTOROS2_TEST_SEQUENCE_MODEL_H

#include <stdint.h>

typedef enum
{
    SEQUENCE_ACCEPT_NEW,
    SEQUENCE_ACCEPT_DUPLICATE,
    SEQUENCE_REJECT_MISMATCH
} SequenceDecision;

typedef struct
{
    int initialized;
    uint16_t last_sequence;
    uint64_t last_payload_hash;
} SequenceModel;

void SequenceModel_Reset(SequenceModel* model);
SequenceDecision SequenceModel_Check(const SequenceModel* model, uint16_t sequence, uint64_t payload_hash);
void SequenceModel_Commit(SequenceModel* model, uint16_t sequence, uint64_t payload_hash);

#endif
