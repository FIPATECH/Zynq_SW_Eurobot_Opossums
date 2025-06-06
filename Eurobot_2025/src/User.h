typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;

#define Abs_Ternaire(a)   ((a<0)?-a:a)
#define Min_Ternaire(a,b) ((a<b)?a:b)
#define Max_Ternaire(a,b) ((a>b)?a:b)
#define sizetab(a) sizeof(a)/sizeof(a[0])
#define Max_Trois(a,b,c) ((a>b)?((a>c)?a:c):((b>c)?b:c))
#define Min_Trois(a,b,c) ((a<b)?((a<c)?a:c):((b<c)?b:c))
#define Max_Quatre(a,b,c,d) ((a>b)?((a>c)?((a>d)?a:d):((c>d)?c:d)):((b>c)?((b>d)?b:d):((c>d)?c:d)))
#define Min_Quatre(a,b,c,d) ((a<b)?((a<c)?((a<d)?a:d):((c<d)?c:d)):((b<c)?((b<d)?b:d):((c<d)?c:d)))