/***************************************************************************

	png.c

    PSP PNG format image I/O functions. (based on M.A.M.E. PNG functions)

***************************************************************************/

#ifndef PSP_PNG_H
#define PSP_PNG_H

int load_png(const char *name, int number);
int save_png(const char *path);

#ifdef NO_INLINE
// formally inline functions
void adjust_blightness(UINT8 *r, UINT8 *g, UINT8 *b);
#endif
#endif /* PSP_PNG_H */
