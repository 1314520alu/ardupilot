/*
 * Appendix B label catalog. Phase-one rows are the approved minimal set.
 * Every other row is documented so it is not mistaken for an implemented channel.
 */
#include "../arinc429.h"

static const a429_label_info_t catalog[] = {
    {0144, A429_DIR_ECU_TO_FC, 1, A429_KIND_BNR, 1, 100, 0, 12050, 0, "Ng"},
    {0145, A429_DIR_ECU_TO_FC, 1, A429_KIND_BNR, 1, 100, 0, 15000, 0, "Np"},
    {0146, A429_DIR_ECU_TO_FC, 0, A429_KIND_BNR, 1, 100, 0, 15000, 0, "Ngc"},
    {0147, A429_DIR_ECU_TO_FC, 1, A429_KIND_BNR, 1, 10, -650, 10600, 0, "T45"},
    {0151, A429_DIR_ECU_TO_FC, 1, A429_KIND_BNR, 1, 10, -300, 8000, 0, "Mkp"},
    {0152, A429_DIR_ECU_TO_FC, 0, A429_KIND_BNR, 1, 10000, 200, 16000, 0, "P3"},
    /* ECU CLP resolution is not given in the appendix ("—"). */
    {0153, A429_DIR_ECU_TO_FC, 0, A429_KIND_BNR, 0, 0, 0, 0, 0, "CLP_ECU"},
    {0154, A429_DIR_ECU_TO_FC, 0, A429_KIND_BNR, 1, 100, -2800, 8800, 0, "LGt"},
    {0155, A429_DIR_ECU_TO_FC, 0, A429_KIND_BNR, 1, 10, -650, 1050, 0, "T1"},
    {0156, A429_DIR_ECU_TO_FC, 0, A429_KIND_BNR, 1, 10000, 0, 1400, 0, "P0_ECU"},
    {0160, A429_DIR_ECU_TO_FC, 0, A429_KIND_BNR, 1, 100, 0, 12050, 0, "NgAc"},
    {0164, A429_DIR_ECU_TO_FC, 0, A429_KIND_DISC, 0, 0, 0, 0, 0, "CmdDiffFlt"},
    {0165, A429_DIR_ECU_TO_FC, 0, A429_KIND_DISC, 0, 0, 0, 0, 0, "ASigFlt"},
    {0166, A429_DIR_ECU_TO_FC, 0, A429_KIND_DISC, 0, 0, 0, 0, 0, "BSigFlt"},
    {0167, A429_DIR_ECU_TO_FC, 0, A429_KIND_DISC, 0, 0, 0, 0, 0, "SigDiffFlt"},
    {0170, A429_DIR_ECU_TO_FC, 0, A429_KIND_DISC, 0, 0, 0, 0, 0, "Swi"},
    {0171, A429_DIR_ECU_TO_FC, 0, A429_KIND_DISC, 0, 0, 0, 0, 0, "Swo"},
    {0172, A429_DIR_ECU_TO_FC, 0, A429_KIND_DISC, 0, 0, 0, 0, 0, "SigFlt"},
    {0173, A429_DIR_ECU_TO_FC, 0, A429_KIND_DISC, 0, 0, 0, 0, 0, "ECUFlt1"},
    {0174, A429_DIR_ECU_TO_FC, 0, A429_KIND_DISC, 0, 0, 0, 0, 0, "ECUFlt2"},
    {0175, A429_DIR_ECU_TO_FC, 1, A429_KIND_DISC, 0, 0, 0, 0, 0, "Warn"},
    {0176, A429_DIR_ECU_TO_FC, 1, A429_KIND_DISC, 0, 0, 0, 0, 0, "State"},
    {0177, A429_DIR_ECU_TO_FC, 1, A429_KIND_DISC, 0, 0, 0, 0, 0, "Warn2"},
    {0200, A429_DIR_ECU_TO_FC, 0, A429_KIND_DISC, 0, 0, 0, 0, 0220, "LIFE"},
    {0310, A429_DIR_FC_TO_ECU, 1, A429_KIND_BNR, 0, 100, 0, 0, 0, "CLP"},
    {0311, A429_DIR_FC_TO_ECU, 1, A429_KIND_BNR, 0, 100, 0, 0, 0, "Nr"},
    {0312, A429_DIR_FC_TO_ECU, 0, A429_KIND_BNR, 0, 100, 0, 0, 0, "P0"},
    {0313, A429_DIR_FC_TO_ECU, 0, A429_KIND_BNR, 0, 10, 0, 0, 0, "T0"},
    {0314, A429_DIR_FC_TO_ECU, 0, A429_KIND_BNR, 0, 1, 0, 0, 0, "H"},
    {0316, A429_DIR_FC_TO_ECU, 0, A429_KIND_BNR, 0, 10, 0, 0, 0, "TAS"},
    {0317, A429_DIR_FC_TO_ECU, 0, A429_KIND_BNR, 0, 10000, 0, 0, 0, "Pm"},
    {0320, A429_DIR_FC_TO_ECU, 0, A429_KIND_BNR, 0, 10, 0, 0, 0, "Tm"},
    {0321, A429_DIR_FC_TO_ECU, 0, A429_KIND_BNR, 0, 1, 0, 0, 0, "Hour"},
    {0322, A429_DIR_FC_TO_ECU, 0, A429_KIND_BNR, 0, 1, 0, 0, 0, "Minute"},
    {0323, A429_DIR_FC_TO_ECU, 0, A429_KIND_BNR, 0, 1, 0, 0, 0, "Second"},
    {0324, A429_DIR_FC_TO_ECU, 0, A429_KIND_BNR, 0, 1, 0, 0, 0, "Year"},
    {0325, A429_DIR_FC_TO_ECU, 0, A429_KIND_BNR, 0, 1, 0, 0, 0, "Month"},
    {0326, A429_DIR_FC_TO_ECU, 0, A429_KIND_BNR, 0, 1, 0, 0, 0, "Day"},
    {0331, A429_DIR_FC_TO_ECU, 0, A429_KIND_DISC, 0, 0, 0, 0, 0, "State331"},
    {0340, A429_DIR_FC_TO_ECU, 1, A429_KIND_DISC, 0, 0, 0, 0, 0, "PCMD1"},
    {0342, A429_DIR_FC_TO_ECU, 1, A429_KIND_DISC, 0, 0, 0, 0, 0, "PCMD2"},
    {0344, A429_DIR_FC_TO_ECU, 1, A429_KIND_DISC, 0, 0, 0, 0, 0, "PCMD3"},
};

const a429_label_info_t *a429_catalog(uint16_t *count)
{
    if (count != 0) {
        *count = (uint16_t)(sizeof(catalog) / sizeof(catalog[0]));
    }
    return catalog;
}

const a429_label_info_t *a429_catalog_find(uint16_t label_oct)
{
    const a429_label_info_t *span = 0;
    uint16_t i;
    uint16_t n = (uint16_t)(sizeof(catalog) / sizeof(catalog[0]));

    for (i = 0; i < n; i++) {
        if (catalog[i].span_last_oct != 0) {
            if (label_oct >= catalog[i].label_oct && label_oct <= catalog[i].span_last_oct) {
                span = &catalog[i];
            }
            continue;
        }
        if (catalog[i].label_oct == label_oct) {
            return &catalog[i];
        }
    }
    return span;
}

int a429_phase1_rx_index(uint16_t label_oct)
{
    switch (label_oct) {
    case 0144:
        return 0;
    case 0145:
        return 1;
    case 0147:
        return 2;
    case 0151:
        return 3;
    case 0175:
        return 4;
    case 0176:
        return 5;
    case 0177:
        return 6;
    default:
        return -1;
    }
}

int a429_pcmd_index(uint16_t label_oct)
{
    switch (label_oct) {
    case 0340:
        return 0;
    case 0342:
        return 1;
    case 0344:
        return 2;
    default:
        return -1;
    }
}
