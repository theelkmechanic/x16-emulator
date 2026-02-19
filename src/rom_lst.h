// Stub ROM listings for TRACE mode
// No official ROM listing files available in this emulator fork.
// Each bank needs: char **lst_bankN
// lst_for_address() indexes these with [address - 0xc000], so NULL pointers
// must be guarded against in the caller (we patch lst_for_address for this).

static char **lst_bank0 = NULL;
static char **lst_bank2 = NULL;
static char **lst_bank3 = NULL;
static char **lst_bank4 = NULL;
static char **lst_bank5 = NULL;
static char **lst_bank7 = NULL;
static char **lst_bank8 = NULL;
static char **lst_bank9 = NULL;
static char **lst_bankA = NULL;
static char **lst_bankB = NULL;
static char **lst_bankC = NULL;
static char **lst_bankD = NULL;
static char **lst_bankE = NULL;
static char **lst_bankF = NULL;
