# LCC - Compilador C Retargetavel: Guia Completo

## Sumario

1. [Introducao](#1-introducao)
2. [Instalacao e Compilacao](#2-instalacao-e-compilacao)
3. [Usando o LCC](#3-usando-o-lcc)
4. [Arquitetura do Compilador](#4-arquitetura-do-compilador)
5. [Representacao Intermediaria](#5-representacao-intermediaria)
6. [O Sistema lburg](#6-o-sistema-lburg)
7. [Criando um Novo Backend](#7-criando-um-novo-backend)
8. [Estrutura Interface](#8-estrutura-interface)
9. [Exemplo Completo: Backend Didatico](#9-exemplo-completo-backend-didatico)
10. [Debugging e Testes](#10-debugging-e-testes)
11. [Otimizacoes](#11-otimizacoes)
12. [Referencia de Arquivos](#12-referencia-de-arquivos)
13. [Solucao de Problemas](#13-solucao-de-problemas)

---

## 1. Introducao

### 1.1 O que e o LCC?

O LCC (Little C Compiler) e um compilador C retargetavel desenvolvido por Chris Fraser e David Hanson. Suas principais caracteristicas sao:

- **Conformidade com C89/ANSI C**: Compila codigo C padrao ANSI
- **Retargetabilidade**: Facilmente adaptavel para novos processadores
- **Codigo Limpo**: Aproximadamente 12.000 linhas de codigo bem documentado
- **Educacional**: Ideal para aprender sobre compiladores
- **Codigo Aberto**: Disponivel para estudo e modificacao

### 1.2 Historico

- **Versao 3.x**: Descrita no livro "A Retargetable C Compiler: Design and Implementation" (1995)
- **Versao 4.x**: Versao atual com mudancas significativas na representacao intermediaria

**IMPORTANTE**: A versao 4.x e incompativel com versoes anteriores.

### 1.3 Arquiteturas Suportadas

| Target | Sistema Operacional |
|--------|---------------------|
| alpha/osf | Alpha, OSF 3.2 |
| mips/irix | MIPS big-endian, IRIX 5.2 |
| sparc/solaris | SPARC, Solaris 2.3 |
| x86/win32 | x86, Windows NT/95/98 |
| x86/linux | x86, Linux |
| **neanderx** | **NEANDER-X 16-bit (educacional)** |
| symbolic | Saida textual da IR (debug) |
| null | Sem saida (testes) |

> **Nota**: Para detalhes especificos sobre o backend NEANDER-X 16-bit, consulte [LCC_NEANDER_BACKEND.md](LCC_NEANDER_BACKEND.md).

---

## 2. Instalacao e Compilacao

### 2.1 Requisitos

- Compilador C (gcc, clang, ou outro compativel com C89)
- Make
- Sistema Unix-like (Linux, macOS, BSD) ou Windows

### 2.2 Obtendo o Codigo

```bash
# Clone ou baixe o repositorio
git clone <url-do-repositorio> lcc
cd lcc
```

### 2.3 Estrutura de Diretorios

```
lcc/
├── src/           # Codigo fonte do compilador (rcc)
├── lburg/         # Gerador de codigo (code-generator generator)
├── cpp/           # Preprocessador C
├── lib/           # Biblioteca runtime
├── include/       # Headers ANSI
├── doc/           # Documentacao
├── alpha/         # Backend Alpha
├── mips/          # Backend MIPS
├── sparc/         # Backend SPARC
├── x86/           # Backend x86
└── etc/           # Scripts de configuracao
```

### 2.4 Compilando o LCC

#### Linux/Unix

```bash
# Edite o arquivo de configuracao para seu sistema
# Exemplo para Linux x86:
cd lcc

# Compile o lburg primeiro
cd lburg
make

# Volte e compile o compilador
cd ..
make HOSTFILE=etc/linux.c

# Ou para um target especifico:
make TARGET=x86/linux
```

#### Compilacao Manual

```bash
# 1. Compile o lburg
cd lburg
cc -o lburg lburg.c gram.c
cd ..

# 2. Gere os arquivos .c dos backends
./lburg/lburg src/x86.md > src/x86.c

# 3. Compile o compilador (rcc)
cc -o rcc src/*.c

# 4. Compile o driver (lcc)
cc -o lcc etc/lcc.c
```

### 2.5 Verificando a Instalacao

```bash
# Verifique se o compilador foi instalado
./lcc -v

# Compile um programa simples
echo 'int main() { return 0; }' > test.c
./lcc -S test.c
cat test.s
```

---

## 3. Usando o LCC

### 3.1 Sintaxe Basica

```bash
lcc [opcoes] arquivo.c ...
```

### 3.2 Opcoes Principais

#### Controle de Compilacao

| Opcao | Descricao |
|-------|-----------|
| `-c` | Compila sem linkar (gera .o ou .obj) |
| `-S` | Gera apenas assembly (.s ou .asm) |
| `-E` | Apenas preprocessa (saida em stdout) |
| `-o arquivo` | Nome do arquivo de saida |

#### Preprocessador

| Opcao | Descricao |
|-------|-----------|
| `-Dnome=valor` | Define macro |
| `-Unome` | Remove definicao de macro |
| `-Idir` | Adiciona diretorio de includes |
| `-N` | Nao busca diretorios padrao de include |

#### Debugging

| Opcao | Descricao |
|-------|-----------|
| `-g` | Gera informacao de debug |
| `-Wf-g1,;` | Debug nivel 1, comentarios com `;` |
| `-v` | Modo verboso (mostra comandos) |
| `-vv` | Mostra comandos mas nao executa |

#### Avisos e Erros

| Opcao | Descricao |
|-------|-----------|
| `-w` | Suprime avisos |
| `-A` | Modo ANSI estrito (avisos extras) |
| `-AA` | Modo ANSI muito estrito |

#### Target

| Opcao | Descricao |
|-------|-----------|
| `-Wf-target=arch/os` | Seleciona arquitetura alvo |

### 3.3 Exemplos de Uso

#### Compilacao Simples

```bash
# Compila e linka
lcc hello.c -o hello

# Apenas compila (gera hello.o)
lcc -c hello.c

# Apenas gera assembly
lcc -S hello.c
```

#### Multiplos Arquivos

```bash
# Compila varios arquivos
lcc main.c utils.c -o programa

# Compila separadamente e linka
lcc -c main.c
lcc -c utils.c
lcc main.o utils.o -o programa
```

#### Usando Target Especifico

```bash
# Gera codigo para x86 Linux
lcc -Wf-target=x86/linux prog.c -S

# Gera representacao simbolica (IR)
lcc -Wf-target=symbolic prog.c -S

# Gera IR como HTML
lcc -Wf-target=symbolic -Wf-html prog.c -S
```

#### Profiling

```bash
# Profiling com bprint
lcc -b prog.c -o prog
./prog
bprint < prof.out

# Profiling com prof
lcc -p prog.c -o prog
./prog
prof prog

# Profiling com gprof
lcc -pg prog.c -o prog
./prog
gprof prog
```

### 3.4 Variaveis de Ambiente

| Variavel | Descricao |
|----------|-----------|
| `LCCINPUTS` | Lista de diretorios para busca de arquivos |
| `LCCDIR` | Diretorio com cpp, rcc e includes |

```bash
# Exemplo
export LCCDIR=/usr/local/lib/lcc
export LCCINPUTS=".:/usr/local/include"
```

---

## 4. Arquitetura do Compilador

### 4.1 Visao Geral

```
                    lcc (driver)
                         |
         +---------------+---------------+
         |               |               |
        cpp             rcc            ld/as
   (preprocessador)  (compilador)   (linker/asm)
```

O LCC e composto por tres programas principais:

1. **cpp**: Preprocessador C
2. **rcc**: Compilador propriamente dito
3. **lcc**: Driver que coordena a compilacao

### 4.2 Fases do Compilador (rcc)

```
Codigo Fonte
     |
     v
+-------------+
| Lexer       |  (lex.c) - Analise lexica
+-------------+
     |
     v
+-------------+
| Parser      |  (decl.c, stmt.c, expr.c) - Analise sintatica
+-------------+
     |
     v
+-------------+
| Type Check  |  (types.c) - Verificacao de tipos
+-------------+
     |
     v
+-------------+
| Tree Gen    |  (tree.c, enode.c) - Arvores de expressao
+-------------+
     |
     v
+-------------+
| DAG Gen     |  (dag.c) - Grafos aciclicos direcionados
+-------------+
     |
     v
+-------------+
| Code Select |  (gen.c + backend) - Selecao de instrucoes
+-------------+
     |
     v
+-------------+
| Reg Alloc   |  (gen.c) - Alocacao de registradores
+-------------+
     |
     v
+-------------+
| Emit        |  (backend) - Emissao de codigo
+-------------+
     |
     v
  Assembly
```

### 4.3 Arquivos Fonte Principais

| Arquivo | Funcao |
|---------|--------|
| `main.c` | Ponto de entrada, inicializacao |
| `lex.c` | Analisador lexico |
| `decl.c` | Declaracoes |
| `stmt.c` | Statements (if, while, for, etc.) |
| `expr.c` | Expressoes |
| `types.c` | Sistema de tipos |
| `sym.c` | Tabela de simbolos |
| `tree.c` | Arvores de expressao |
| `enode.c` | Nos de expressao |
| `dag.c` | Construcao de DAGs |
| `gen.c` | Geracao de codigo, alocacao de registradores |
| `output.c` | Saida formatada |
| `alloc.c` | Alocacao de memoria |
| `string.c` | Manipulacao de strings |
| `simp.c` | Simplificacoes algebraicas |
| `init.c` | Inicializadores |
| `error.c` | Mensagens de erro |
| `c.h` | Header principal com todas as definicoes |

### 4.4 Fluxo de Dados

```c
// Exemplo: x = a + b * c;

// 1. Lexer produz tokens:
//    ID(x) ASSIGN ID(a) PLUS ID(b) STAR ID(c) SEMI

// 2. Parser constroi AST:
//    ASGN
//    /   \
//   x    ADD
//        / \
//       a  MUL
//          / \
//         b   c

// 3. DAG elimina subexpressoes comuns e gera IR:
//    t1 = MULI4(b, c)
//    t2 = ADDI4(a, t1)
//    ASGNI4(x, t2)

// 4. Code Selection escolhe instrucoes:
//    mov eax, [b]
//    imul eax, [c]
//    add eax, [a]
//    mov [x], eax
```

---

## 5. Representacao Intermediaria

### 5.1 Operacoes IR

O LCC usa uma representacao intermediaria baseada em arvores/DAGs. As operacoes sao definidas em `src/ops.h`:

#### Formato das Operacoes

Cada operacao tem o formato: `OPERACAO + TIPO + TAMANHO`

**Tipos**:
- `I` - Signed integer
- `U` - Unsigned integer
- `F` - Floating point
- `P` - Pointer
- `V` - Void
- `B` - Block (struct)

**Tamanhos**:
- `1` - 1 byte (char)
- `2` - 2 bytes (short)
- `4` - 4 bytes (int/float)
- `8` - 8 bytes (long long/double)

#### Categorias de Operacoes

**Constantes e Enderecos**:
```
CNST   - Constante (CNSTI4 = constante int de 4 bytes)
ADDRG  - Endereco de global
ADDRF  - Endereco de parametro (frame)
ADDRL  - Endereco de local
```

**Acesso a Memoria**:
```
INDIR  - Indirection (load)
ASGN   - Assignment (store)
```

**Aritmetica**:
```
ADD    - Adicao
SUB    - Subtracao
MUL    - Multiplicacao
DIV    - Divisao
MOD    - Modulo
NEG    - Negacao
```

**Logica e Bits**:
```
BAND   - AND bit a bit
BOR    - OR bit a bit
BXOR   - XOR bit a bit
BCOM   - Complemento (NOT)
LSH    - Left shift
RSH    - Right shift
```

**Conversao**:
```
CVF    - Convert from float
CVI    - Convert from int
CVU    - Convert from unsigned
CVP    - Convert from pointer
```

**Comparacao**:
```
EQ     - Equal
NE     - Not equal
LT     - Less than
LE     - Less or equal
GT     - Greater than
GE     - Greater or equal
```

**Controle de Fluxo**:
```
JUMP   - Salto incondicional
LABEL  - Rotulo
CALL   - Chamada de funcao
RET    - Retorno
ARG    - Argumento de funcao
```

### 5.2 Estrutura Node

```c
struct node {
    short op;           // Operacao (ex: ADDI4)
    short count;        // Contagem de referencias
    Symbol syms[3];     // Simbolos associados
    Node kids[2];       // Filhos (operandos)
    Node link;          // Proximo no na lista
    Xnode x;            // Dados especificos do backend
};
```

### 5.3 Exemplo de IR

Para o codigo C:
```c
int foo(int a, int b) {
    return a + b * 2;
}
```

A IR gerada (com `-Wf-target=symbolic`):

```
function foo ncalls=0
  caller a type=int sclass=AUTO offset=0
  caller b type=int sclass=AUTO offset=4
  callee a type=int sclass=AUTO offset=0
  callee b type=int sclass=AUTO offset=4
  1. INDIRI4 ADDRLP4 b
  2. CNSTI4 2
  3. MULI4 #1 #2
  4. INDIRI4 ADDRLP4 a
  5. ADDI4 #4 #3
  6. RETI4 #5
```

---

## 6. O Sistema lburg

### 6.1 O que e lburg?

O `lburg` (LCC's BURG) e um "gerador de gerador de codigo". Ele le uma especificacao de maquina (arquivo `.md`) e produz um seletor de instrucoes otimo baseado em programacao dinamica.

BURG = Bottom-Up Rewrite Grammar

### 6.2 Formato do Arquivo .md

```
%{
/* Secao de configuracao C */
#include "c.h"
/* Definicoes, prototipos, variaveis */
%}

%start stmt           /* Simbolo inicial */

%term ADDI4=4405     /* Declaracao de terminais */
%term SUBI4=4421

%%
/* Regras de producao */
nonterminal: pattern  "template"  custo

%%
/* Codigo C adicional */
```

### 6.3 Componentes

#### Secao de Configuracao

Define macros obrigatorias:

```c
%{
#define NODEPTR_TYPE Node       // Tipo de ponteiro para no
#define OP_LABEL(p) ((p)->op)   // Obtem operacao do no
#define LEFT_CHILD(p) ((p)->kids[0])   // Filho esquerdo
#define RIGHT_CHILD(p) ((p)->kids[1])  // Filho direito
#define STATE_LABEL(p) ((p)->x.state)  // Estado do no
%}
```

#### Declaracao de Terminais

```c
%term ADDI4=4405    // ADD + I + 4 = 19*256 + 4 + 1*16 + 5
%term CNSTI4=4117
%term INDIRI4=4165
```

O numero e calculado: `(opcode << 8) + (type << 4) + size`

#### Regras de Producao

Formato:
```
nonterminal: padrao  "template"  [custo]
```

Exemplos:
```c
// Carregar constante em registrador
reg: CNSTI4  "mov %c,%0\n"  1

// Adicao de registrador com memoria/constante
reg: ADDI4(reg,mrc)  "?mov %c,%0\nadd %c,%1\n"  1

// Armazenar em memoria
stmt: ASGNI4(addr,reg)  "mov %0,%1\n"  1
```

### 6.4 Templates de Saida

**Especificadores**:

| Codigo | Significado |
|--------|-------------|
| `%0` | Primeiro filho |
| `%1` | Segundo filho |
| `%c` | Registrador resultado |
| `%a` | Simbolo (nome/valor) |
| `%F` | Framesize |
| `%%` | Literal `%` |
| `?` | Emite se destino != origem |
| `#` | Comentario (nao emite) |

### 6.5 Custos

- Custos sao inteiros nao-negativos
- Custo 0 e default
- O lburg escolhe a derivacao de menor custo total
- Custos podem ser expressoes C:

```c
reg: LOADI4(reg)  "# move\n"  move(a)  // Funcao calcula custo
stmt: ASGNI4(addr,ADDI4(mem,rc))  "add %1,%2\n"  memop(a)
```

### 6.6 Nao-terminais Comuns

```c
reg     - Valor em registrador
stmt    - Statement (sem valor)
con     - Constante
addr    - Endereco de memoria
mem     - Operando em memoria
rc      - Registrador ou constante
mr      - Memoria ou registrador
mrc     - Memoria, registrador ou constante
```

### 6.7 Exemplo Completo

```c
%{
#include "c.h"
#define NODEPTR_TYPE Node
#define OP_LABEL(p) ((p)->op)
#define LEFT_CHILD(p) ((p)->kids[0])
#define RIGHT_CHILD(p) ((p)->kids[1])
#define STATE_LABEL(p) ((p)->x.state)

static void progbeg(int, char**);
static void progend(void);
/* ... mais declaracoes ... */
%}

%start stmt

%term CNSTI4=4117
%term ADDI4=4405
%term ASGNI4=4149
%term INDIRI4=4165
%term ADDRLP4=4391

%%

/* Constantes */
con: CNSTI4  "%a"

/* Enderecos */
addr: ADDRLP4  "%a[ebp]"

/* Carga de memoria */
reg: INDIRI4(addr)  "mov %c,%0\n"  1

/* Constante para registrador */
reg: con  "mov %c,%0\n"  1

/* Adicao */
reg: ADDI4(reg,reg)  "?mov %c,%0\nadd %c,%1\n"  1
reg: ADDI4(reg,con)  "?mov %c,%0\nadd %c,%1\n"  1

/* Armazenamento */
stmt: ASGNI4(addr,reg)  "mov %0,%1\n"  1

/* Statement vazio de registrador */
stmt: reg  ""

%%

/* Implementacao das funcoes do backend */
static void progbeg(int argc, char *argv[]) {
    /* Inicializacao */
}

static void progend(void) {
    /* Finalizacao */
}
```

### 6.8 Usando lburg

```bash
# Gera codigo C a partir da especificacao
./lburg/lburg src/x86.md > src/x86.c

# Com prefixo customizado
./lburg/lburg -p myprefix_ src/myarch.md > src/myarch.c

# Com tracing (debug)
./lburg/lburg -T src/x86.md > src/x86.c
```

---

## 7. Criando um Novo Backend

### 7.1 Passos Gerais

1. Criar arquivo de especificacao `.md`
2. Implementar a estrutura `Interface`
3. Registrar o backend em `bind.c`
4. Compilar e testar

### 7.2 Planejamento

Antes de comecar, determine:

1. **Caracteristicas do processador**:
   - Tamanho de palavra
   - Registradores disponiveis
   - Modos de enderecamento
   - Conjunto de instrucoes

2. **Tipos de dados**:
   - Tamanhos (char, short, int, long, ponteiros)
   - Alinhamentos
   - Ordem dos bytes (endianness)

3. **Convencao de chamada**:
   - Passagem de parametros
   - Valor de retorno
   - Registradores preservados

### 7.3 Estrutura do Arquivo .md

```c
%{
/* =========================================
 * Backend para Processador XYZ
 * ========================================= */

#include "c.h"

/* Macros obrigatorias */
#define NODEPTR_TYPE Node
#define OP_LABEL(p) ((p)->op)
#define LEFT_CHILD(p) ((p)->kids[0])
#define RIGHT_CHILD(p) ((p)->kids[1])
#define STATE_LABEL(p) ((p)->x.state)

/* Registradores */
enum { R0=0, R1=1, R2=2, R3=3, SP=13, LR=14, PC=15 };

/* Prototipos */
static void address(Symbol, Symbol, long);
static void blockbeg(Env *);
static void blockend(Env *);
static void defaddress(Symbol);
static void defconst(int, int, Value);
static void defstring(int, char *);
static void defsymbol(Symbol);
static void doarg(Node);
static void emit2(Node);
static void export(Symbol);
static void clobber(Node);
static void function(Symbol, Symbol [], Symbol [], int);
static void global(Symbol);
static void import(Symbol);
static void local(Symbol);
static void progbeg(int, char **);
static void progend(void);
static void segment(int);
static void space(int);
static void target(Node);

/* Variaveis */
static int cseg;  /* Segmento atual */
static Symbol intreg[32];  /* Registradores inteiros */
static Symbol intregw;     /* Wildcard */

%}

%start stmt

/* Terminais - apenas os necessarios */
%term CNSTI1=1045 CNSTI2=2069 CNSTI4=4117
%term CNSTU1=1046 CNSTU2=2070 CNSTU4=4118
%term CNSTP4=4119

%term ARGB=41
%term ARGI4=4133 ARGP4=4135 ARGU4=4134

%term ASGNB=57
%term ASGNI1=1077 ASGNI2=2101 ASGNI4=4149
%term ASGNU1=1078 ASGNU2=2102 ASGNU4=4150
%term ASGNP4=4151

%term INDIRB=73
%term INDIRI1=1093 INDIRI2=2117 INDIRI4=4165
%term INDIRU1=1094 INDIRU2=2118 INDIRU4=4166
%term INDIRP4=4167

%term CVII1=1157 CVII2=2181 CVII4=4229
%term CVIU1=1158 CVIU2=2182 CVIU4=4230
%term CVUI1=1205 CVUI2=2229 CVUI4=4277
%term CVUU1=1206 CVUU2=2230 CVUU4=4278
%term CVUP4=4279 CVPU4=4246

%term NEGI4=4293

%term CALLI4=4309 CALLP4=4311 CALLU4=4310 CALLV=216

%term RETI4=4341 RETP4=4343 RETU4=4342 RETV=248

%term ADDRGP4=4359
%term ADDRFP4=4375
%term ADDRLP4=4391

%term ADDI4=4405 ADDP4=4407 ADDU4=4406
%term SUBI4=4421 SUBP4=4423 SUBU4=4422

%term LSHI4=4437 LSHU4=4438
%term RSHI4=4469 RSHU4=4470

%term MODI4=4453 MODU4=4454
%term DIVI4=4485 DIVU4=4486
%term MULI4=4501 MULU4=4502

%term BANDI4=4517 BANDU4=4518
%term BCOMI4=4533 BCOMU4=4534
%term BORI4=4549  BORU4=4550
%term BXORI4=4565 BXORU4=4566

%term EQI4=4549 EQU4=4550
%term GEI4=4565 GEU4=4566
%term GTI4=4581 GTU4=4582
%term LEI4=4597 LEU4=4598
%term LTI4=4613 LTU4=4614
%term NEI4=4629 NEU4=4630

%term JUMPV=584
%term LABELV=600

%term LOADI1=1253 LOADI2=2277 LOADI4=4325
%term LOADU1=1254 LOADU2=2278 LOADU4=4326
%term LOADP4=4327

%term VREGP=711

%%

/* =====================
 * REGRAS DE PRODUCAO
 * ===================== */

/* Registradores virtuais */
reg: INDIRI1(VREGP)  "# read reg\n"
reg: INDIRU1(VREGP)  "# read reg\n"
reg: INDIRI2(VREGP)  "# read reg\n"
reg: INDIRU2(VREGP)  "# read reg\n"
reg: INDIRI4(VREGP)  "# read reg\n"
reg: INDIRU4(VREGP)  "# read reg\n"
reg: INDIRP4(VREGP)  "# read reg\n"

stmt: ASGNI1(VREGP,reg)  "# write reg\n"
stmt: ASGNU1(VREGP,reg)  "# write reg\n"
stmt: ASGNI2(VREGP,reg)  "# write reg\n"
stmt: ASGNU2(VREGP,reg)  "# write reg\n"
stmt: ASGNI4(VREGP,reg)  "# write reg\n"
stmt: ASGNU4(VREGP,reg)  "# write reg\n"
stmt: ASGNP4(VREGP,reg)  "# write reg\n"

/* Constantes */
con: CNSTI1  "%a"
con: CNSTU1  "%a"
con: CNSTI2  "%a"
con: CNSTU2  "%a"
con: CNSTI4  "%a"
con: CNSTU4  "%a"
con: CNSTP4  "%a"

/* Enderecos */
addr: ADDRGP4  "%a"
addr: ADDRFP4  "[fp, #%a]"
addr: ADDRLP4  "[fp, #%a]"

/* Carga de memoria */
reg: INDIRI4(addr)  "ldr %c, %0\n"  1
reg: INDIRU4(addr)  "ldr %c, %0\n"  1
reg: INDIRP4(addr)  "ldr %c, %0\n"  1
reg: INDIRI1(addr)  "ldrsb %c, %0\n"  1
reg: INDIRU1(addr)  "ldrb %c, %0\n"  1
reg: INDIRI2(addr)  "ldrsh %c, %0\n"  1
reg: INDIRU2(addr)  "ldrh %c, %0\n"  1

/* Constante para registrador */
reg: con  "mov %c, #%0\n"  1

/* Armazenamento */
stmt: ASGNI4(addr,reg)  "str %1, %0\n"  1
stmt: ASGNU4(addr,reg)  "str %1, %0\n"  1
stmt: ASGNP4(addr,reg)  "str %1, %0\n"  1
stmt: ASGNI1(addr,reg)  "strb %1, %0\n"  1
stmt: ASGNU1(addr,reg)  "strb %1, %0\n"  1
stmt: ASGNI2(addr,reg)  "strh %1, %0\n"  1
stmt: ASGNU2(addr,reg)  "strh %1, %0\n"  1

/* Aritmetica */
reg: ADDI4(reg,reg)  "add %c, %0, %1\n"  1
reg: ADDU4(reg,reg)  "add %c, %0, %1\n"  1
reg: ADDP4(reg,reg)  "add %c, %0, %1\n"  1

reg: SUBI4(reg,reg)  "sub %c, %0, %1\n"  1
reg: SUBU4(reg,reg)  "sub %c, %0, %1\n"  1
reg: SUBP4(reg,reg)  "sub %c, %0, %1\n"  1

reg: MULI4(reg,reg)  "mul %c, %0, %1\n"  10
reg: MULU4(reg,reg)  "mul %c, %0, %1\n"  10

reg: DIVI4(reg,reg)  "sdiv %c, %0, %1\n"  20
reg: DIVU4(reg,reg)  "udiv %c, %0, %1\n"  20

/* Logicas */
reg: BANDI4(reg,reg)  "and %c, %0, %1\n"  1
reg: BANDU4(reg,reg)  "and %c, %0, %1\n"  1
reg: BORI4(reg,reg)   "orr %c, %0, %1\n"  1
reg: BORU4(reg,reg)   "orr %c, %0, %1\n"  1
reg: BXORI4(reg,reg)  "eor %c, %0, %1\n"  1
reg: BXORU4(reg,reg)  "eor %c, %0, %1\n"  1

reg: BCOMI4(reg)  "mvn %c, %0\n"  1
reg: BCOMU4(reg)  "mvn %c, %0\n"  1
reg: NEGI4(reg)   "neg %c, %0\n"  1

/* Shifts */
reg: LSHI4(reg,reg)  "lsl %c, %0, %1\n"  1
reg: LSHU4(reg,reg)  "lsl %c, %0, %1\n"  1
reg: RSHI4(reg,reg)  "asr %c, %0, %1\n"  1
reg: RSHU4(reg,reg)  "lsr %c, %0, %1\n"  1

/* Comparacoes */
stmt: EQI4(reg,reg)  "cmp %0, %1\nbeq %a\n"  1
stmt: NEI4(reg,reg)  "cmp %0, %1\nbne %a\n"  1
stmt: LTI4(reg,reg)  "cmp %0, %1\nblt %a\n"  1
stmt: LEI4(reg,reg)  "cmp %0, %1\nble %a\n"  1
stmt: GTI4(reg,reg)  "cmp %0, %1\nbgt %a\n"  1
stmt: GEI4(reg,reg)  "cmp %0, %1\nbge %a\n"  1

stmt: LTU4(reg,reg)  "cmp %0, %1\nblo %a\n"  1
stmt: LEU4(reg,reg)  "cmp %0, %1\nbls %a\n"  1
stmt: GTU4(reg,reg)  "cmp %0, %1\nbhi %a\n"  1
stmt: GEU4(reg,reg)  "cmp %0, %1\nbhs %a\n"  1

/* Saltos */
stmt: JUMPV(addr)  "b %0\n"  1
stmt: LABELV  "%a:\n"

/* Chamadas */
reg: CALLI4(addr)  "bl %0\n"  1
reg: CALLU4(addr)  "bl %0\n"  1
reg: CALLP4(addr)  "bl %0\n"  1
stmt: CALLV(addr)  "bl %0\n"  1

/* Argumentos */
stmt: ARGI4(reg)  "# arg\n"  1
stmt: ARGU4(reg)  "# arg\n"  1
stmt: ARGP4(reg)  "# arg\n"  1

/* Retorno */
stmt: RETI4(reg)  "# ret\n"  1
stmt: RETU4(reg)  "# ret\n"  1
stmt: RETP4(reg)  "# ret\n"  1
stmt: RETV        "# ret\n"

/* Conversoes */
reg: CVII4(reg)  "# cvt\n"  1
reg: CVUI4(reg)  "# cvt\n"  1
reg: CVIU4(reg)  "# cvt\n"  1
reg: CVUU4(reg)  "# cvt\n"  1

/* Moves */
reg: LOADI4(reg)  "mov %c, %0\n"  move(a)
reg: LOADU4(reg)  "mov %c, %0\n"  move(a)
reg: LOADP4(reg)  "mov %c, %0\n"  move(a)

/* Statement vazio */
stmt: reg  ""

%%

/* =====================
 * IMPLEMENTACAO
 * ===================== */

/* Implementacao das funcoes... */
/* Ver secao 8 para detalhes */
```

---

## 8. Estrutura Interface

### 8.1 Definicao Completa

A estrutura `Interface` (definida em `src/c.h`) e o contrato entre o frontend e o backend:

```c
typedef struct interface {
    /* Metricas de tipos: tamanho, alinhamento, outofline */
    Metrics charmetric;
    Metrics shortmetric;
    Metrics intmetric;
    Metrics longmetric;
    Metrics longlongmetric;
    Metrics floatmetric;
    Metrics doublemetric;
    Metrics longdoublemetric;
    Metrics ptrmetric;
    Metrics structmetric;

    /* Flags de configuracao */
    unsigned little_endian:1;
    unsigned mulops_calls:1;
    unsigned wants_callb:1;
    unsigned wants_argb:1;
    unsigned left_to_right:1;
    unsigned wants_dag:1;
    unsigned unsigned_char:1;

    /* Funcoes de callback */
    void (*address)(Symbol p, Symbol q, long n);
    void (*blockbeg)(Env *);
    void (*blockend)(Env *);
    void (*defaddress)(Symbol);
    void (*defconst)(int suffix, int size, Value v);
    void (*defstring)(int n, char *s);
    void (*defsymbol)(Symbol);
    void (*emit)(Node);
    void (*export)(Symbol);
    void (*function)(Symbol, Symbol[], Symbol[], int);
    Node (*gen)(Node);
    void (*global)(Symbol);
    void (*import)(Symbol);
    void (*local)(Symbol);
    void (*progbeg)(int argc, char *argv[]);
    void (*progend)(void);
    void (*segment)(int);
    void (*space)(int);

    /* Funcoes de debug (podem ser NULL) */
    void (*stabblock)(int, int, Symbol*);
    void (*stabend)(Coordinate *, Symbol, Coordinate **, Symbol *, Symbol *);
    void (*stabfend)(Symbol, int);
    void (*stabinit)(char *, int, char *[]);
    void (*stabline)(Coordinate *);
    void (*stabsym)(Symbol);
    void (*stabtype)(Symbol);

    /* Extensao especifica do backend */
    Xinterface x;
} Interface;
```

### 8.2 Metricas de Tipos

```c
typedef struct metrics {
    unsigned char size;      /* Tamanho em bytes */
    unsigned char align;     /* Alinhamento em bytes */
    unsigned char outofline; /* 1 se operacoes sao out-of-line */
} Metrics;
```

Exemplo para x86:
```c
Interface x86IR = {
    1, 1, 0,  /* char:   size=1, align=1, outofline=0 */
    2, 2, 0,  /* short:  size=2, align=2 */
    4, 4, 0,  /* int:    size=4, align=4 */
    4, 4, 0,  /* long:   size=4, align=4 */
    4, 4, 0,  /* long long: (nao suportado corretamente) */
    4, 4, 1,  /* float:  size=4, align=4, outofline=1 */
    8, 4, 1,  /* double: size=8, align=4, outofline=1 */
    8, 4, 1,  /* long double */
    4, 4, 0,  /* T*:     size=4, align=4 */
    0, 1, 0,  /* struct: size=0 (calculado), align=1 */
    ...
};
```

### 8.3 Flags de Configuracao

| Flag | Descricao |
|------|-----------|
| `little_endian` | 1 se little-endian, 0 se big-endian |
| `mulops_calls` | 1 se MUL/DIV/MOD sao chamadas de funcao |
| `wants_callb` | 1 se quer instrucao CALLB para struct |
| `wants_argb` | 1 se quer instrucao ARGB para struct |
| `left_to_right` | 1 se argumentos avaliados esquerda->direita |
| `wants_dag` | 1 se quer manter estrutura DAG |
| `unsigned_char` | 1 se char e unsigned por padrao |

### 8.4 Funcoes Callback

#### progbeg / progend

```c
/* Chamada no inicio do programa */
static void progbeg(int argc, char *argv[]) {
    /* Processar argumentos do compilador */
    for (int i = 0; i < argc; i++) {
        if (strcmp(argv[i], "-g") == 0) {
            /* Habilitar debug */
        }
    }

    /* Inicializar registradores */
    intreg[0] = mkreg("r0", 0, 1, IREG);
    intreg[1] = mkreg("r1", 1, 1, IREG);
    /* ... */

    /* Emitir cabecalho do assembly */
    print(".text\n");
}

/* Chamada no fim do programa */
static void progend(void) {
    print(".end\n");
}
```

#### segment

```c
/* Troca segmento (TEXT, DATA, BSS, LIT) */
static void segment(int seg) {
    if (seg == cseg) return;
    cseg = seg;
    switch (seg) {
    case CODE: print(".text\n"); break;
    case DATA: print(".data\n"); break;
    case BSS:  print(".bss\n");  break;
    case LIT:  print(".rodata\n"); break;
    }
}
```

#### defsymbol

```c
/* Define nome externo do simbolo */
static void defsymbol(Symbol p) {
    if (p->scope >= LOCAL && p->sclass == STATIC)
        p->x.name = stringf(".L%d", genlabel(1));
    else if (p->generated)
        p->x.name = stringf(".L%s", p->name);
    else if (p->scope == GLOBAL || p->sclass == EXTERN)
        p->x.name = stringf("_%s", p->name);  /* Prefixo _ */
    else
        p->x.name = p->name;
}
```

#### global

```c
/* Declara variavel global */
static void global(Symbol p) {
    print(".align %d\n", p->type->align);
    print("%s:\n", p->x.name);
}
```

#### export / import

```c
/* Exporta simbolo */
static void export(Symbol p) {
    print(".global %s\n", p->x.name);
}

/* Importa simbolo */
static void import(Symbol p) {
    if (p->ref > 0)
        print(".extern %s\n", p->x.name);
}
```

#### defconst / defaddress / defstring / space

```c
/* Define constante */
static void defconst(int suffix, int size, Value v) {
    switch (size) {
    case 1: print(".byte %d\n", v.u & 0xFF); break;
    case 2: print(".short %d\n", v.i & 0xFFFF); break;
    case 4: print(".long %d\n", v.i); break;
    }
}

/* Define endereco */
static void defaddress(Symbol p) {
    print(".long %s\n", p->x.name);
}

/* Define string */
static void defstring(int len, char *s) {
    for (int i = 0; i < len; i++)
        print(".byte %d\n", s[i] & 0xFF);
}

/* Reserva espaco */
static void space(int n) {
    print(".space %d\n", n);
}
```

#### function

```c
/* Gera prologo e epilogo de funcao */
static void function(Symbol f, Symbol caller[], Symbol callee[], int ncalls) {
    int i;

    /* Emite rotulo */
    print("%s:\n", f->x.name);

    /* Prologo */
    print("    push {fp, lr}\n");
    print("    mov fp, sp\n");

    /* Configura offsets dos parametros */
    offset = 8;  /* Apos fp e lr salvos */
    for (i = 0; callee[i]; i++) {
        Symbol p = callee[i];
        Symbol q = caller[i];
        p->x.offset = q->x.offset = offset;
        p->x.name = q->x.name = stringf("%d", offset);
        offset += roundup(q->type->size, 4);
    }

    /* Reseta offset para locais */
    offset = maxoffset = 0;

    /* Gera codigo do corpo */
    gencode(caller, callee);

    /* Aloca espaco para locais */
    framesize = roundup(maxoffset, 8);
    if (framesize > 0)
        print("    sub sp, sp, #%d\n", framesize);

    /* Emite codigo */
    emitcode();

    /* Epilogo */
    print("    mov sp, fp\n");
    print("    pop {fp, pc}\n");
}
```

#### local

```c
/* Aloca variavel local */
static void local(Symbol p) {
    /* Tenta alocar em registrador */
    if (askregvar(p, rmap(ttob(p->type))) == 0) {
        /* Aloca na pilha */
        offset = roundup(offset + p->type->size, p->type->align);
        p->x.offset = -offset;
        p->x.name = stringf("%d", -offset);
    }
}
```

#### blockbeg / blockend

```c
/* Inicio de bloco */
static void blockbeg(Env *e) {
    e->offset = offset;
}

/* Fim de bloco */
static void blockend(Env *e) {
    if (offset > maxoffset)
        maxoffset = offset;
    offset = e->offset;
}
```

#### address

```c
/* Calcula endereco p + n */
static void address(Symbol q, Symbol p, long n) {
    if (p->scope == GLOBAL || p->sclass == STATIC || p->sclass == EXTERN)
        q->x.name = stringf("%s%s%D", p->x.name, n >= 0 ? "+" : "", n);
    else {
        q->x.offset = p->x.offset + n;
        q->x.name = stringf("%d", q->x.offset);
    }
}
```

### 8.5 Extensao Xinterface

```c
typedef struct {
    unsigned char max_unaligned_load;
    Symbol (*rmap)(int);  /* Mapeia tipo para classe de reg */

    /* Funcoes para block copy */
    void (*blkfetch)(int size, int off, int reg, int tmp);
    void (*blkstore)(int size, int off, int reg, int tmp);
    void (*blkloop)(int dreg, int doff, int sreg, int soff,
                    int size, int tmps[]);

    /* Funcoes geradas pelo lburg */
    void (*_label)(Node);
    int (*_rule)(void*, int);
    short **_nts;
    Node (*_kids)(Node, int, Node*);
    char **_string;
    char **_templates;
    char *_isinstruction;
    char **_ntname;

    /* Funcoes do backend */
    void (*emit2)(Node);
    void (*doarg)(Node);
    void (*target)(Node);
    void (*clobber)(Node);
} Xinterface;
```

---

## 9. Exemplo Completo: Backend Didatico

### 9.1 Processador Hipotetico "SIMPLE"

Caracteristicas:
- 8 registradores de 32 bits: R0-R7
- R7 = Stack Pointer (SP)
- R6 = Link Register (LR)
- Instrucoes de 3 operandos
- Memoria enderecada por byte

### 9.2 Arquivo simple.md

```c
%{
/* =============================================
 * Backend para processador SIMPLE
 * ============================================= */

#include "c.h"

#define NODEPTR_TYPE Node
#define OP_LABEL(p) ((p)->op)
#define LEFT_CHILD(p) ((p)->kids[0])
#define RIGHT_CHILD(p) ((p)->kids[1])
#define STATE_LABEL(p) ((p)->x.state)

/* Registradores */
enum { R0=0, R1, R2, R3, R4, R5, LR=6, SP=7 };
#define IREG 0
#define FREG 1

/* Prototipos */
static void address(Symbol, Symbol, long);
static void blockbeg(Env *);
static void blockend(Env *);
static void defaddress(Symbol);
static void defconst(int, int, Value);
static void defstring(int, char *);
static void defsymbol(Symbol);
static void doarg(Node);
static void emit2(Node);
static void export(Symbol);
static void clobber(Node);
static void function(Symbol, Symbol [], Symbol [], int);
static void global(Symbol);
static void import(Symbol);
static void local(Symbol);
static void progbeg(int, char **);
static void progend(void);
static void segment(int);
static void space(int);
static void target(Node);
static Symbol rmap(int);
static void blkfetch(int, int, int, int);
static void blkstore(int, int, int, int);
static void blkloop(int, int, int, int, int, int[]);

/* Variaveis globais */
static int cseg = 0;
static Symbol intreg[32];
static Symbol intregw;
static int argno;

%}

%start stmt

/* Terminais */
%term CNSTI4=4117 CNSTU4=4118 CNSTP4=4119

%term ARGI4=4133 ARGU4=4134 ARGP4=4135

%term ASGNI1=1077 ASGNI2=2101 ASGNI4=4149
%term ASGNU1=1078 ASGNU2=2102 ASGNU4=4150
%term ASGNP4=4151

%term INDIRI1=1093 INDIRI2=2117 INDIRI4=4165
%term INDIRU1=1094 INDIRU2=2118 INDIRU4=4166
%term INDIRP4=4167

%term CVII4=4229 CVUI4=4277 CVIU4=4230 CVUU4=4278

%term NEGI4=4293

%term CALLI4=4309 CALLU4=4310 CALLP4=4311 CALLV=216

%term RETI4=4341 RETU4=4342 RETP4=4343 RETV=248

%term ADDRGP4=4359 ADDRFP4=4375 ADDRLP4=4391

%term ADDI4=4405 ADDU4=4406 ADDP4=4407
%term SUBI4=4421 SUBU4=4422 SUBP4=4423
%term MULI4=4501 MULU4=4502
%term DIVI4=4485 DIVU4=4486
%term MODI4=4453 MODU4=4454

%term BANDI4=4517 BANDU4=4518
%term BORI4=4549  BORU4=4550
%term BXORI4=4565 BXORU4=4566
%term BCOMI4=4533 BCOMU4=4534

%term LSHI4=4437 LSHU4=4438
%term RSHI4=4469 RSHU4=4470

%term EQI4=4581  EQU4=4582
%term NEI4=4597  NEU4=4598
%term LTI4=4613  LTU4=4614
%term LEI4=4629  LEU4=4630
%term GTI4=4645  GTU4=4646
%term GEI4=4661  GEU4=4662

%term JUMPV=584
%term LABELV=600

%term LOADI4=4325 LOADU4=4326 LOADP4=4327

%term VREGP=711

%%

/* Registradores virtuais */
reg: INDIRI4(VREGP)  "# read reg\n"
reg: INDIRU4(VREGP)  "# read reg\n"
reg: INDIRP4(VREGP)  "# read reg\n"

stmt: ASGNI4(VREGP,reg)  "# write reg\n"
stmt: ASGNU4(VREGP,reg)  "# write reg\n"
stmt: ASGNP4(VREGP,reg)  "# write reg\n"

/* Constantes */
con: CNSTI4  "%a"
con: CNSTU4  "%a"
con: CNSTP4  "%a"

/* Enderecos */
addr: ADDRGP4  "%a"
addr: ADDRFP4  "[R7, #%a]"
addr: ADDRLP4  "[R7, #%a]"

/* Carga de memoria */
reg: INDIRI4(addr)  "LOAD %c, %0\n"  1
reg: INDIRU4(addr)  "LOAD %c, %0\n"  1
reg: INDIRP4(addr)  "LOAD %c, %0\n"  1

/* Constante para registrador */
reg: con  "MOVI %c, %0\n"  1

/* Armazenamento */
stmt: ASGNI4(addr,reg)  "STORE %1, %0\n"  1
stmt: ASGNU4(addr,reg)  "STORE %1, %0\n"  1
stmt: ASGNP4(addr,reg)  "STORE %1, %0\n"  1

/* Aritmetica */
reg: ADDI4(reg,reg)  "ADD %c, %0, %1\n"  1
reg: ADDU4(reg,reg)  "ADD %c, %0, %1\n"  1
reg: ADDP4(reg,reg)  "ADD %c, %0, %1\n"  1

reg: ADDI4(reg,con)  "ADDI %c, %0, %1\n"  1
reg: ADDU4(reg,con)  "ADDI %c, %0, %1\n"  1
reg: ADDP4(reg,con)  "ADDI %c, %0, %1\n"  1

reg: SUBI4(reg,reg)  "SUB %c, %0, %1\n"  1
reg: SUBU4(reg,reg)  "SUB %c, %0, %1\n"  1
reg: SUBP4(reg,reg)  "SUB %c, %0, %1\n"  1

reg: MULI4(reg,reg)  "MUL %c, %0, %1\n"  10
reg: MULU4(reg,reg)  "MUL %c, %0, %1\n"  10

reg: DIVI4(reg,reg)  "DIV %c, %0, %1\n"  20
reg: DIVU4(reg,reg)  "DIVU %c, %0, %1\n"  20

reg: MODI4(reg,reg)  "MOD %c, %0, %1\n"  20
reg: MODU4(reg,reg)  "MODU %c, %0, %1\n"  20

/* Operacoes logicas */
reg: BANDI4(reg,reg)  "AND %c, %0, %1\n"  1
reg: BANDU4(reg,reg)  "AND %c, %0, %1\n"  1

reg: BORI4(reg,reg)   "OR %c, %0, %1\n"   1
reg: BORU4(reg,reg)   "OR %c, %0, %1\n"   1

reg: BXORI4(reg,reg)  "XOR %c, %0, %1\n"  1
reg: BXORU4(reg,reg)  "XOR %c, %0, %1\n"  1

reg: BCOMI4(reg)      "NOT %c, %0\n"      1
reg: BCOMU4(reg)      "NOT %c, %0\n"      1

reg: NEGI4(reg)       "NEG %c, %0\n"      1

/* Shifts */
reg: LSHI4(reg,reg)  "SHL %c, %0, %1\n"  1
reg: LSHU4(reg,reg)  "SHL %c, %0, %1\n"  1
reg: RSHI4(reg,reg)  "SAR %c, %0, %1\n"  1
reg: RSHU4(reg,reg)  "SHR %c, %0, %1\n"  1

/* Comparacoes e saltos condicionais */
stmt: EQI4(reg,reg)  "CMP %0, %1\nJEQ %a\n"  2
stmt: EQU4(reg,reg)  "CMP %0, %1\nJEQ %a\n"  2
stmt: NEI4(reg,reg)  "CMP %0, %1\nJNE %a\n"  2
stmt: NEU4(reg,reg)  "CMP %0, %1\nJNE %a\n"  2
stmt: LTI4(reg,reg)  "CMP %0, %1\nJLT %a\n"  2
stmt: LTU4(reg,reg)  "CMP %0, %1\nJLO %a\n"  2
stmt: LEI4(reg,reg)  "CMP %0, %1\nJLE %a\n"  2
stmt: LEU4(reg,reg)  "CMP %0, %1\nJLS %a\n"  2
stmt: GTI4(reg,reg)  "CMP %0, %1\nJGT %a\n"  2
stmt: GTU4(reg,reg)  "CMP %0, %1\nJHI %a\n"  2
stmt: GEI4(reg,reg)  "CMP %0, %1\nJGE %a\n"  2
stmt: GEU4(reg,reg)  "CMP %0, %1\nJHS %a\n"  2

/* Salto incondicional */
stmt: JUMPV(addr)  "JMP %0\n"  1
stmt: JUMPV(reg)   "JMPR %0\n"  1

/* Labels */
stmt: LABELV  "%a:\n"

/* Chamada de funcao */
reg: CALLI4(addr)  "CALL %0\n"  1
reg: CALLU4(addr)  "CALL %0\n"  1
reg: CALLP4(addr)  "CALL %0\n"  1
stmt: CALLV(addr)  "CALL %0\n"  1

/* Argumentos */
stmt: ARGI4(reg)  "PUSH %0\n"  1
stmt: ARGU4(reg)  "PUSH %0\n"  1
stmt: ARGP4(reg)  "PUSH %0\n"  1

/* Retorno */
stmt: RETI4(reg)  "MOV R0, %0\n"  1
stmt: RETU4(reg)  "MOV R0, %0\n"  1
stmt: RETP4(reg)  "MOV R0, %0\n"  1
stmt: RETV        "# ret void\n"

/* Conversoes */
reg: CVII4(reg)  "# cvt\n"  1
reg: CVUI4(reg)  "# cvt\n"  1
reg: CVIU4(reg)  "# cvt\n"  1
reg: CVUU4(reg)  "# cvt\n"  1

/* Moves */
reg: LOADI4(reg)  "MOV %c, %0\n"  move(a)
reg: LOADU4(reg)  "MOV %c, %0\n"  move(a)
reg: LOADP4(reg)  "MOV %c, %0\n"  move(a)

/* Statement vazio */
stmt: reg  ""

%%

/* ============================================
 * IMPLEMENTACAO DAS FUNCOES DO BACKEND
 * ============================================ */

static Symbol rmap(int opk) {
    return intregw;
}

static void blkfetch(int k, int off, int reg, int tmp) {}
static void blkstore(int k, int off, int reg, int tmp) {}
static void blkloop(int dreg, int doff, int sreg, int soff,
                    int size, int tmps[]) {}

static void progbeg(int argc, char *argv[]) {
    int i;

    /* Processar opcoes */
    for (i = 1; i < argc; i++) {
        /* Opcoes especificas do backend */
    }

    /* Criar registradores */
    for (i = 0; i < 6; i++)
        intreg[i] = mkreg("R%d", i, 1, IREG);

    intregw = mkwildcard(intreg);

    /* Mascara de registradores livres */
    tmask[IREG] = (1<<6) - 1;  /* R0-R5 */
    vmask[IREG] = 0;

    print("; Gerado por LCC para SIMPLE\n");
    print(".text\n");
}

static void progend(void) {
    print(".end\n");
}

static void segment(int seg) {
    if (cseg == seg) return;
    cseg = seg;
    switch (seg) {
    case CODE: print(".text\n"); break;
    case DATA: print(".data\n"); break;
    case BSS:  print(".bss\n");  break;
    case LIT:  print(".rodata\n"); break;
    }
}

static void defsymbol(Symbol p) {
    if (p->scope >= LOCAL && p->sclass == STATIC)
        p->x.name = stringf(".L%d", genlabel(1));
    else if (p->generated)
        p->x.name = stringf(".L%s", p->name);
    else if (p->scope == GLOBAL || p->sclass == EXTERN)
        p->x.name = stringf("_%s", p->name);
    else
        p->x.name = p->name;
}

static void address(Symbol q, Symbol p, long n) {
    if (p->scope == GLOBAL || p->sclass == STATIC || p->sclass == EXTERN)
        q->x.name = stringf("%s%s%D", p->x.name, n >= 0 ? "+" : "", n);
    else {
        q->x.offset = p->x.offset + n;
        q->x.name = stringf("%d", q->x.offset);
    }
}

static void defconst(int suffix, int size, Value v) {
    switch (size) {
    case 1: print(".byte %d\n", v.u & 0xFF); break;
    case 2: print(".half %d\n", v.i & 0xFFFF); break;
    case 4: print(".word %d\n", v.i); break;
    default: assert(0);
    }
}

static void defaddress(Symbol p) {
    print(".word %s\n", p->x.name);
}

static void defstring(int len, char *s) {
    int i;
    for (i = 0; i < len; i++)
        print(".byte %d\n", s[i] & 0xFF);
}

static void export(Symbol p) {
    print(".global %s\n", p->x.name);
}

static void import(Symbol p) {
    if (p->ref > 0)
        print(".extern %s\n", p->x.name);
}

static void global(Symbol p) {
    print(".align %d\n", p->type->align);
    print("%s:\n", p->x.name);
}

static void space(int n) {
    print(".space %d\n", n);
}

static void blockbeg(Env *e) {
    e->offset = offset;
}

static void blockend(Env *e) {
    if (offset > maxoffset)
        maxoffset = offset;
    offset = e->offset;
}

static void local(Symbol p) {
    if (askregvar(p, rmap(ttob(p->type))) == 0) {
        offset = roundup(offset + p->type->size, p->type->align);
        p->x.offset = -offset;
        p->x.name = stringf("%d", -offset);
    }
}

static void function(Symbol f, Symbol caller[], Symbol callee[], int ncalls) {
    int i;

    /* Rotulo da funcao */
    print("\n; Function %s\n", f->name);
    print("%s:\n", f->x.name);

    /* Prologo */
    print("    PUSH R6        ; Salva LR\n");
    print("    MOV R6, R7     ; Frame pointer\n");

    /* Inicializa mascaras de registradores */
    usedmask[IREG] = 0;
    freemask[IREG] = tmask[IREG];

    /* Configura parametros */
    offset = 8;  /* Apos LR e FP salvos */
    for (i = 0; callee[i]; i++) {
        Symbol p = callee[i];
        Symbol q = caller[i];
        p->x.offset = q->x.offset = offset;
        p->x.name = q->x.name = stringf("%d", offset);
        p->sclass = q->sclass = AUTO;
        offset += roundup(q->type->size, 4);
    }

    /* Gera codigo */
    offset = maxoffset = argoffset = maxargoffset = 0;
    gencode(caller, callee);

    /* Aloca espaco para locais */
    framesize = roundup(maxoffset + maxargoffset, 8);
    if (framesize > 0)
        print("    SUBI R7, R7, %d  ; Aloca frame\n", framesize);

    /* Emite codigo gerado */
    emitcode();

    /* Epilogo */
    print("    MOV R7, R6     ; Restaura SP\n");
    print("    POP R6         ; Restaura LR\n");
    print("    RET            ; Retorna\n");
}

static void emit2(Node p) {
    /* Emissao especial se necessario */
}

static void doarg(Node p) {
    argno++;
}

static void target(Node p) {
    switch (specific(p->op)) {
    case CALL+I:
    case CALL+U:
    case CALL+P:
    case CALL+V:
        setreg(p, intreg[0]);  /* Retorno em R0 */
        break;
    case RET+I:
    case RET+U:
    case RET+P:
        rtarget(p, 0, intreg[0]);  /* Valor em R0 */
        break;
    }
}

static void clobber(Node p) {
    switch (specific(p->op)) {
    case CALL+I:
    case CALL+U:
    case CALL+P:
    case CALL+V:
        /* Chamadas destroem R0-R3 */
        spill(0x0F, IREG, p);
        break;
    }
}

/* Definicao da Interface */
Interface simpleIR = {
    /* Metricas de tipos */
    1, 1, 0,  /* char */
    2, 2, 0,  /* short */
    4, 4, 0,  /* int */
    4, 4, 0,  /* long */
    4, 4, 0,  /* long long */
    4, 4, 1,  /* float (outofline) */
    8, 4, 1,  /* double */
    8, 4, 1,  /* long double */
    4, 4, 0,  /* T* */
    0, 1, 0,  /* struct */

    /* Flags */
    1,  /* little_endian */
    0,  /* mulops_calls */
    0,  /* wants_callb */
    1,  /* wants_argb */
    0,  /* left_to_right */
    0,  /* wants_dag */
    0,  /* unsigned_char */

    /* Callbacks */
    address,
    blockbeg,
    blockend,
    defaddress,
    defconst,
    defstring,
    defsymbol,
    emit,
    export,
    function,
    gen,
    global,
    import,
    local,
    progbeg,
    progend,
    segment,
    space,

    /* Debug (NULL) */
    0, 0, 0, 0, 0, 0, 0,

    /* Xinterface */
    {
        1,      /* max_unaligned_load */
        rmap,
        blkfetch, blkstore, blkloop,
        _label,
        _rule,
        _nts,
        _kids,
        _string,
        _templates,
        _isinstruction,
        _ntname,
        emit2,
        doarg,
        target,
        clobber,
    }
};
```

### 9.3 Registrando o Backend

Adicione em `src/bind.c`:

```c
#define yy \
xx(alpha/osf,    alphaIR) \
xx(mips/irix,    mipsebIR) \
xx(sparc/sun,    sparcIR) \
xx(x86/win32,    x86IR) \
xx(x86/linux,    x86linuxIR) \
xx(simple,       simpleIR) \     /* NOVO */
xx(symbolic,     symbolicIR) \
xx(null,         nullIR)
```

### 9.4 Compilando

```bash
# Gera o .c a partir do .md
./lburg/lburg src/simple.md > src/simple.c

# Compila o compilador com o novo backend
cc -c src/simple.c -o src/simple.o
cc -o rcc src/*.o

# Testa
echo 'int main() { return 42; }' > test.c
./rcc -target=simple test.c
```

---

## 10. Debugging e Testes

### 10.1 Opcoes de Debug

```bash
# Mostra comandos executados
lcc -v prog.c

# Mostra mas nao executa
lcc -vv prog.c

# Gera IR simbolica
lcc -Wf-target=symbolic prog.c -S

# IR como HTML
lcc -Wf-target=symbolic -Wf-html prog.c -S

# Debug com source nos comentarios
lcc -Wf-g1,# prog.c -S
```

### 10.2 Backend Simbolico

O backend `symbolic` imprime a representacao intermediaria em formato legivel:

```bash
./rcc -target=symbolic test.c
```

Saida exemplo:
```
progbeg
function main ncalls=0
  caller (empty)
  callee (empty)
  1. CNSTI4 42
  2. RETI4 #1
progend
```

### 10.3 Testando o Backend

#### Testes Unitarios

```c
/* test_arith.c */
int add(int a, int b) { return a + b; }
int sub(int a, int b) { return a - b; }
int mul(int a, int b) { return a * b; }
int div(int a, int b) { return a / b; }

int main() {
    int x = 10;
    int y = 3;
    return add(x, y) - sub(x, y) + mul(x, y) * div(x, y);
}
```

```bash
# Compila e verifica assembly
./lcc -Wf-target=simple -S test_arith.c
cat test_arith.s
```

#### Testes de Controle de Fluxo

```c
/* test_control.c */
int factorial(int n) {
    if (n <= 1) return 1;
    return n * factorial(n - 1);
}

int main() {
    int sum = 0;
    int i;
    for (i = 0; i < 10; i++) {
        sum += i;
    }
    return sum + factorial(5);
}
```

### 10.4 Ferramentas de Debug

#### Dump de Arvore

```c
/* Em src/tree.c */
void printtree(Tree tp, int depth) {
    int i;
    for (i = 0; i < depth; i++) print("  ");
    print("%s", opname(tp->op));
    if (tp->kids[0]) {
        print("\n");
        printtree(tp->kids[0], depth + 1);
    }
    if (tp->kids[1]) {
        print("\n");
        printtree(tp->kids[1], depth + 1);
    }
}
```

#### Tracing do lburg

```bash
# Compile lburg com tracing
./lburg/lburg -T src/myarch.md > src/myarch.c
```

---

## 11. Otimizacoes

### 11.1 Otimizacoes no Frontend

O LCC realiza otimizacoes limitadas no frontend:

- **Constant folding**: `3 + 4` -> `7`
- **Strength reduction**: `x * 2` -> `x + x` ou `x << 1`
- **Algebraic simplification**: `x + 0` -> `x`

### 11.2 Otimizacoes via Custos

Use custos no `.md` para preferir instrucoes melhores:

```c
/* Preferir shift a multiplicacao por potencia de 2 */
reg: MULI4(reg,con2)  "SHL %c, %0, 1\n"  1   /* custo 1 */
reg: MULI4(reg,reg)   "MUL %c, %0, %1\n"  10  /* custo 10 */

/* Incremento in-place */
stmt: ASGNI4(addr,ADDI4(INDIRI4(addr),con1))  "INC %0\n"  1
```

### 11.3 Otimizacoes de Peephole

Implemente em `emit2`:

```c
static void emit2(Node p) {
    static Node prev = NULL;

    /* Remove MOV redundante */
    if (prev && specific(prev->op) == LOAD+I
        && specific(p->op) == LOAD+I
        && prev->syms[RX] == p->x.kids[0]->syms[RX]) {
        /* MOV R1, R0; MOV R2, R1 -> MOV R2, R0 */
        print("MOV %s, %s\n", p->syms[RX]->x.name,
              prev->x.kids[0]->syms[RX]->x.name);
        prev = p;
        return;
    }

    /* Emissao normal */
    prev = p;
}
```

### 11.4 Alocacao de Registradores

O LCC usa um alocador de registradores baseado em graph coloring simplificado:

- Variaveis com mais referencias tem prioridade
- Use `askregvar()` para tentar alocar variaveis em registradores
- Use `rtarget()` e `setreg()` para direcionar resultados

```c
static void local(Symbol p) {
    /* Tenta alocar em registrador */
    if (askregvar(p, rmap(ttob(p->type))) == 0) {
        /* Falhou - aloca na pilha */
        offset = roundup(offset + p->type->size, p->type->align);
        p->x.offset = -offset;
        p->x.name = stringf("%d", -offset);
    }
}
```

---

## 12. Referencia de Arquivos

### 12.1 Arquivos Fonte (src/)

| Arquivo | Linhas | Descricao |
|---------|--------|-----------|
| c.h | ~600 | Header principal, todas as definicoes |
| main.c | ~150 | Ponto de entrada |
| lex.c | ~500 | Analisador lexico |
| decl.c | ~1000 | Declaracoes |
| stmt.c | ~400 | Statements |
| expr.c | ~500 | Expressoes |
| types.c | ~500 | Sistema de tipos |
| sym.c | ~300 | Tabela de simbolos |
| tree.c | ~200 | Arvores de expressao |
| enode.c | ~200 | Nos de expressao |
| dag.c | ~500 | Construcao de DAGs |
| gen.c | ~800 | Geracao de codigo |
| simp.c | ~200 | Simplificacoes |
| output.c | ~100 | Saida formatada |
| alloc.c | ~100 | Alocacao de memoria |
| string.c | ~100 | Strings |
| init.c | ~200 | Inicializadores |
| error.c | ~100 | Erros |
| bind.c | ~30 | Registro de backends |
| ops.h | ~130 | Definicao de operacoes IR |

### 12.2 Backends (src/)

| Arquivo | Target |
|---------|--------|
| x86.md | x86 Windows |
| x86linux.md | x86 Linux |
| alpha.md | Alpha OSF |
| mips.md | MIPS IRIX |
| sparc.md | SPARC Solaris |
| symbolic.c | Debug (IR textual) |
| bytecode.c | Bytecode |
| null.c | Sem saida |

### 12.3 lburg (lburg/)

| Arquivo | Descricao |
|---------|-----------|
| lburg.c | Principal |
| lburg.h | Definicoes |
| gram.y | Gramatica yacc |
| gram.c | Parser gerado |

---

## 13. Solucao de Problemas

### 13.1 Erros Comuns

#### "unknown target"

```
lcc: unknown target 'xyz'; must specify one of
    -target=alpha/osf
    -target=x86/linux
    ...
```

**Solucao**: Use um target valido ou registre seu backend em `bind.c`.

#### "expression too complicated"

```
error: expression too complicated
```

**Solucao**: Simplifique a expressao ou aumente o limite de profundidade no backend.

#### Erro de custo infinito

```
no cover for tree
```

**Solucao**: Adicione regras para o padrao faltante no arquivo `.md`.

### 13.2 Debugging de Regras

Se uma regra nao esta sendo selecionada:

1. Verifique se os terminais estao declarados
2. Verifique se os custos estao corretos
3. Use `-T` no lburg para tracing

### 13.3 Problemas de Registradores

Se ha spills excessivos:

1. Verifique `tmask` e `vmask`
2. Verifique se `askregvar()` esta sendo chamado
3. Verifique a funcao `clobber()`

### 13.4 Convencao de Chamada

Se funcoes nao funcionam:

1. Verifique `function()` - prologo/epilogo
2. Verifique `target()` - registrador de retorno
3. Verifique `doarg()` - passagem de parametros

---

## Apendice A: Calculando Codigos de Terminal

O codigo de terminal e calculado como:

```
codigo = (opcode << 4) | tipo_size
```

Onde:
- opcode: numero da operacao (ex: ADD=19)
- tipo: I=1, U=2, P=3, F=4, etc.
- size: 1, 2, 4, 8

Exemplo:
```
ADDI4 = (19 << 4) | 4 | (1 << 8) = 4405
       = 0x1135
```

Use `src/ops.h` como referencia.

---

## Apendice B: Macros Uteis

```c
/* Obtem operacao generica */
#define generic(op)  ((op) & 0x3F0)

/* Obtem tipo da operacao */
#define optype(op)   ((op) >> 4 & 0xF)

/* Obtem tamanho da operacao */
#define opsize(op)   ((op) & 0xF)

/* Verifica se e operacao de endereco */
#define isaddrop(op) (generic(op) == ADDRG || \
                      generic(op) == ADDRF || \
                      generic(op) == ADDRL)

/* Arredonda para alinhamento */
#define roundup(x,n) (((x)+((n)-1))&(~((n)-1)))
```

---

## Apendice C: Referencias

1. Fraser, C. W. & Hanson, D. R. **"A Retargetable C Compiler: Design and Implementation"**. Addison-Wesley, 1995. ISBN 0-8053-1670-1

2. Fraser, C. W., Hanson, D. R. & Proebsting, T. A. **"Engineering a Simple, Efficient Code Generator Generator"**. ACM LOPLAS, 1992.

3. Documentacao online: https://drh.github.io/lcc/

4. Interface 4.x: https://drh.github.io/lcc/documents/interface4.pdf

5. **Backend NEANDER-X 16-bit**: [LCC_NEANDER_BACKEND.md](LCC_NEANDER_BACKEND.md) - Documentacao especifica para o processador educacional NEANDER-X

---

## Licenca

O LCC e distribuido sob uma licenca propria. Veja o arquivo `CPYRIGHT` para detalhes sobre uso, modificacao e distribuicao.

---

*Documento gerado para fins educacionais. Para informacoes atualizadas, consulte a documentacao oficial do LCC.*
