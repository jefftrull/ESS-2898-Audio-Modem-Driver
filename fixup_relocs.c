/*
 * a program to clean up troublesome relocation records in .o files
 * by using BFD to add/delete/edit them
 * 
 * Copyright (C) 2006 Jeffrey Elliot Trull
 *
 * Significant parts copied from objcopy.c in binutils, so:
 *
 * Copyright 1991, 1992, 1993, 1994, 1995, 1996, 1997, 1998, 1999, 2000,
 * 2001, 2002, 2003, 2004, 2005
 * Free Software Foundation, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/* I want strndup */
#define _GNU_SOURCE

#include <bfd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <mcheck.h>

/* global variables hold the things we are supposed to do to the obj file */

/* relocation records we are to ADD */
typedef struct add_reloc_struct {
    char *symbol_name;
    unsigned long loc;
    struct add_reloc_struct *next;
} add_reloc_struct;
add_reloc_struct *additional_relocs = NULL;

/* relocation records we are to DELETE */
typedef struct delete_reloc_struct {
    char *symbol_name;
    struct delete_reloc_struct *next;
} delete_reloc_struct;
delete_reloc_struct *delete_relocs = NULL;

/* relocation records we are to CHANGE */
typedef struct change_reloc_struct {
    char *old_symbol_name;
    char *new_symbol_name;
    struct change_reloc_struct *next;
} change_reloc_struct;
change_reloc_struct *change_relocs = NULL;

/* modifications to data requested */
typedef struct modify_byte_struct {
    unsigned long location;
    unsigned long value;
    struct modify_byte_struct *next;
} modify_byte_struct;
modify_byte_struct *modify_bytes = NULL;

static asymbol **isympp = NULL;	/* Input symbols.  */

static void setup_section (bfd *, asection *, void *);
static void setup_bfd_headers (bfd *, bfd *);
static void copy_section_relocs_edit (bfd *, asection *, void *);
static void copy_section_data (bfd *, asection *, void *);

void process_section_relocs (bfd *ibfd, asection *sect, bfd *obfd) {
    long relsize, relcount;
    arelent **relpp;
    asymbol **sympp;

    if (strcmp(bfd_get_section_name(ibfd, sect), ".text") == 0) {
	long i;
	relsize = bfd_get_reloc_upper_bound (ibfd, sect);
	sympp = (asymbol **)malloc(bfd_get_symtab_upper_bound(ibfd));
	bfd_canonicalize_symtab (ibfd, sympp);
	relpp = (arelent **)malloc ((size_t)relsize);
	relcount = bfd_canonicalize_reloc (ibfd, sect, relpp, sympp);

	/* create a new relocation array for the output */

	/* loop through the reloc records */
	for (i = 0; i < relcount; i++) {
	    printf("%-30s%10x%10x%10s\n", bfd_asymbol_name (*relpp[i]->sym_ptr_ptr),
		   relpp[i]->address, relpp[i]->addend, relpp[i]->howto->name);
	}
    }
}  
	       
void print_text_section_relocs (bfd *abfd, asection *sect, PTR some_object) {
    long relsize, relcount;
    arelent **relpp;
    asymbol **sympp;

    if (strcmp(bfd_get_section_name(abfd, sect), ".text") == 0) {
	long i;
	relsize = bfd_get_reloc_upper_bound (abfd, sect);
	sympp = (asymbol **)malloc(bfd_get_symtab_upper_bound(abfd));
	bfd_canonicalize_symtab (abfd, sympp);
	printf("found text section of size %x\n", bfd_get_section_size(sect));
	relpp = (arelent **)malloc ((size_t)relsize);
	relcount = bfd_canonicalize_reloc (abfd, sect, relpp, sympp);
	printf("reloc records are as follows:\n");
	for (i = 0; i < relcount; i++) {
	    printf("%-30s%10x%10x%10s\n", bfd_asymbol_name (*relpp[i]->sym_ptr_ptr),
		   relpp[i]->address, relpp[i]->addend, relpp[i]->howto->name);
	}
    }
    else {
	printf("section name %s is not the same as \".text\"\n", bfd_get_section_name(abfd, sect));
    }
}  
	       

/* find and return a change command relating to a particular reloc */
struct change_reloc_struct *find_change_reloc (const char *name, change_reloc_struct *change_list_head) {
    change_reloc_struct *change_ptr;

    for (change_ptr = change_list_head; change_ptr != NULL; change_ptr = change_ptr->next) {
	if (strcmp(name, change_ptr->old_symbol_name) == 0) {
	    return change_ptr;
	}
    }
    return NULL;
}

main (int argc, char **argv) {
    bfd *ibfd, *obfd;
    asection *p;
    static asymbol **osympp, **delsympp;
    long symsize, symcount, delsymcount;
    int i;
    int c;
    int idx;
    struct add_reloc_struct *new_reloc;
    struct change_reloc_struct *new_change;
    struct delete_reloc_struct *new_delete;
    struct modify_byte_struct *new_modify;

    while ((c = getopt (argc, argv, "a:d:c:m:")) != -1) {
	switch (c) {
	    case 'a':
		/* check to see if we have two args: name and loc */
		if ((index(optarg, ',') == NULL) ||
		    (index(optarg, ',') != rindex(optarg, ','))) {
		    fprintf(stderr, "usage: -a argument should be <symbolname>,<location>, not \"%s\"\n", optarg);
		    exit(1);
		}
		/* record the add reloc command in the global array */
		new_reloc = (add_reloc_struct *)malloc(sizeof(add_reloc_struct));
		new_reloc->symbol_name = strndup(optarg, (index(optarg, ',') - optarg));
		new_reloc->loc = strtol(index(optarg, ',') + 1, NULL, 0);
		if (errno == EINVAL) {
		    fprintf(stderr, "the value %s is not a valid location for the add command\n", index(optarg, ',') + 1);
		    exit(1);
		}
		new_reloc->next = additional_relocs;
		additional_relocs = new_reloc;
		break;

	    case 'c':
		/* check to see if we have two args */
		if ((index(optarg, ',') == NULL) ||
		    (index(optarg, ',') != rindex(optarg, ','))) {
		    fprintf(stderr, "usage: -c argument should be <symbolname>,<symbolname>, not \"%s\"\n", optarg);
		    exit(1);
		}
		new_change = (change_reloc_struct *)malloc(sizeof(change_reloc_struct));
		new_change->old_symbol_name = strndup(optarg, strlen(optarg) - strlen(index(optarg, ',')));
		new_change->new_symbol_name = strdup(index(optarg, ',') + 1);
		new_change->next = change_relocs;
		change_relocs = new_change;
		break;

	    case 'd':
		new_delete = (delete_reloc_struct *)malloc(sizeof(delete_reloc_struct));
		new_delete->symbol_name = strdup(optarg);
		new_delete->next = delete_relocs;
		delete_relocs = new_delete;
		break;

	    case 'm':
		if ((index(optarg, '=') == NULL) ||
		    (index(optarg, '=') != rindex(optarg, '='))) {
		    fprintf(stderr, "usage: -m argument should be <location>=<value>, not \"%s\"\n", optarg);
		    exit(1);
		}
		new_modify = (modify_byte_struct *)malloc(sizeof(modify_byte_struct));
		new_modify->location = strtol(optarg, NULL, 0);
		new_modify->value = strtol(index(optarg, '=') + 1, NULL, 0);
		if (new_modify->value > 0xff) {
		    fprintf(stderr, "requested modify value %x for location %x exceeds 0xff\n");
		    exit(1);
		}
		new_modify->next = modify_bytes;
		modify_bytes = new_modify;
		break;

	    default:
		fprintf(stderr, "unrecognized argument character |%c|\n", c);
	}
    }

    if ((argc - optind) != 2) {
	fprintf(stderr, "usage: fixup_relocs [-a newsymbol,location] [-c oldsymbol,newsymbol] [-d symbol] infile.o outfile.o\n");
	exit(1);
    }

/*
    printf("performing the following changes:\n\n");
    printf("DEL relocs:\n");
    struct delete_reloc_struct *del_ptr;
    for (del_ptr = delete_relocs; del_ptr != NULL; del_ptr = del_ptr->next) {
	printf("%s\n", del_ptr->symbol_name);
    }
    printf("\nCHANGE relocs:\n");
    struct change_reloc_struct *change_ptr;
    for (change_ptr = change_relocs; change_ptr != NULL; change_ptr = change_ptr->next) {
	printf("%-30s->%30s\n", change_ptr->old_symbol_name, change_ptr->new_symbol_name);
    }
    printf("\nADD relocs:\n");
    struct add_reloc_struct *add_ptr;
    for (add_ptr = additional_relocs; add_ptr != NULL; add_ptr = add_ptr->next) {
	printf("%-30s @%30x\n", add_ptr->symbol_name, add_ptr->loc);
    }
*/
    ibfd = bfd_openr(argv[optind], NULL);
    if (ibfd == NULL) {
	bfd_perror("while opening input object file");
	exit(1);
    }

    /* if I don't do "check_format", there's no data in the bfd object.  wtf? */
    if (!bfd_check_format(ibfd, bfd_object)) {
	fprintf(stderr, "input file %x seems to NOT be an object file! exiting.\n", argv[optind]);
	exit(1);
    }

    obfd = bfd_openw(argv[optind+1], bfd_get_target(ibfd));
    if (obfd == NULL) {
	bfd_perror("while opening output object file");
	exit(1);
    }

    if (!bfd_set_format(obfd, bfd_get_format(ibfd))) {
	bfd_perror("while setting output object file format");
    }

    /* copy a bunch of necessary global stuff */
    bfd_set_start_address(obfd, bfd_get_start_address(ibfd));
    bfd_set_file_flags(obfd, bfd_get_file_flags(ibfd));
    bfd_set_arch_mach(obfd, bfd_get_arch(ibfd), bfd_get_mach(ibfd));
    /* BOZO objcopy sets format again at this point.  why? */

    bfd_map_over_sections (ibfd, setup_section, obfd);

    setup_bfd_headers (ibfd, obfd);

    /* Symbol filtering must happen after the output sections
       have been created, but before their contents are set.  */
    symsize = bfd_get_symtab_upper_bound (ibfd);
    if (symsize < 0) {
	fprintf(stderr, "problem processing %s\n", bfd_get_filename (ibfd));
	return FALSE;
    }

    /* count the added relocations so we can put extra space in the output symbol table for them */
    int reloc_add_cnt, reloc_delete_cnt;
    reloc_add_cnt = 0;
    reloc_delete_cnt = 0;
    for (new_reloc = additional_relocs; new_reloc != NULL; new_reloc = new_reloc->next) {
	reloc_add_cnt++;
    }
    /* the "change" symbols might also not be in the symbol table yet */
    for (new_change = change_relocs; new_change != NULL; new_change = new_change->next) {
	reloc_add_cnt++;
	/* the old symbol may be deleted, also */
	reloc_delete_cnt++;
    }
    for (new_delete = delete_relocs; new_delete != NULL; new_delete = new_delete->next) {
	reloc_delete_cnt++;
    }

    /* filter symbol table in two steps: */
    /* 1) move symbols bound for deletion to the end of the output symbol table array */
    /* 2) truncate the table at the first of those */
    /* this makes it possible to do the reloc processing with the symbol table intact, */
    /* and remove the deleted symbols afterwards, without corrupting the reloc data structures */
    isympp = malloc (symsize);
    osympp = malloc (symsize + reloc_add_cnt * sizeof(asymbol *));
    delsympp = malloc (reloc_delete_cnt * sizeof(asymbol *));
    symcount = bfd_canonicalize_symtab (ibfd, isympp);

    if (symcount < 0) {
	fprintf(stderr, "problem processing %s\n", bfd_get_filename (ibfd));
	return FALSE;
    }

    /* remove any undefined symbols whose relocation entries were deleted or changed */
    int osym_idx, delsym_idx;
    osym_idx = delsym_idx = 0;
    delsymcount = 0;
    for (i = 0; i < symcount; i++) {
	if ((is_delete_reloc(bfd_asymbol_name(isympp[i]), delete_relocs) ||
	     (find_change_reloc(bfd_asymbol_name(isympp[i]), change_relocs) != NULL)) &&
	    (isympp[i]->section != NULL) &&
	    (strcmp(isympp[i]->section->name, BFD_UND_SECTION_NAME) == 0)) {
	    delsympp[delsym_idx++] = isympp[i];
	}
	else {
	    osympp[osym_idx++] = isympp[i];
	}
    }
    symcount = osym_idx;
    delsymcount = delsym_idx;
    osympp[symcount] = NULL;

    /* add the symbols for additional relocs to the table */
    int added_symbol_cnt = 0;
    for (new_reloc = additional_relocs; new_reloc != NULL; new_reloc = new_reloc->next) {
	if (find_symbol(osympp, new_reloc->symbol_name) < 0) {
	    /* not yet present, so add it */
	    asymbol *new_sym;
	    new_sym = bfd_make_empty_symbol(obfd);
	    new_sym->name = strdup(new_reloc->symbol_name);
	    new_sym->section = bfd_get_section_by_name (obfd, ".text");
	    new_sym->value = new_reloc->loc;
	    new_sym->flags = BSF_GLOBAL;
	    printf("adding symbol %s (%x) to output symbol table at index %d\n", new_reloc->symbol_name, new_sym, symcount + added_symbol_cnt);
	    osympp[symcount + added_symbol_cnt++] = new_sym;
	    osympp[symcount + added_symbol_cnt] = NULL;
	}
    }

    /* do the same for changed relocs */
    for (new_change = change_relocs; new_change != NULL; new_change = new_change->next) {
	if (find_symbol(osympp, new_change->new_symbol_name) < 0) {
	    /* not yet present, so add it */
	    /* since this is a name change, we will reuse the existing address (value field of reloc) */
	    int old_symbol_idx;
	    if ((old_symbol_idx = find_symbol(isympp, new_change->old_symbol_name)) < 0) {
		fprintf(stderr, "change command old symbol name %s not found in symbol table! Exiting.\n", new_change->old_symbol_name);
		exit(1);
	    }
	    asymbol *new_sym;
	    new_sym = bfd_make_empty_symbol(obfd);
	    new_sym->name = strdup(new_change->new_symbol_name);
	    new_sym->section = bfd_und_section_ptr;
	    new_sym->value = isympp[old_symbol_idx]->value;
	    new_sym->flags = BSF_GLOBAL;
	    fprintf(stderr, "adding new symbol %s for change reloc command\n", new_sym->name);
	    osympp[symcount + added_symbol_cnt++] = new_sym;
	    osympp[symcount + added_symbol_cnt] = NULL;
	}
    }

    /* append the soon-to-be deleted symbols to the end of the output symbol table */
    for (i = 0; i < delsymcount; i++) {
	osympp[symcount + added_symbol_cnt + i] = delsympp[i];
    }
    osympp[symcount + added_symbol_cnt + delsymcount] = NULL;

    bfd_set_symtab (obfd, osympp, symcount + added_symbol_cnt + delsymcount);

    /* This has to happen after the symbol table has been set.  */
    bfd_map_over_sections (ibfd, copy_section_relocs_edit, obfd);

    /* now truncate the symbol table to eliminate the deleted symbols */
    osympp[symcount + added_symbol_cnt] = NULL;

    bfd_set_symtab (obfd, osympp, symcount + added_symbol_cnt);
    
    /* now that we've set the relocs and cleaned the symtab, can call this */
    bfd_map_over_sections (ibfd, copy_section_data, obfd);

    bfd_close(obfd);
    bfd_close(ibfd);

    return 0;

}

/* Once each of the sections is copied, we may still need to do some
   finalization work for private section headers.  Do that here.  */

static void
setup_bfd_headers (bfd *ibfd, bfd *obfd)
{
  const char *err;

  /* Allow the BFD backend to copy any private data it understands
     from the input section to the output section.  */
  if (! bfd_copy_private_header_data (ibfd, obfd))
    {
      perror("private header data");
    }

  /* All went well.  */
  return;

  fprintf(stderr, "%s: error in %s: %s",
	     bfd_get_filename (ibfd),
	     err, bfd_errmsg (bfd_get_error ()));
}

/* Create a section in OBFD with the same
   name and attributes as ISECTION in IBFD.  */

static void
setup_section (bfd *ibfd, sec_ptr isection, void *obfdarg)
{
  bfd *obfd = obfdarg;
  struct section_list *p;
  sec_ptr osection;
  bfd_size_type size;
  bfd_vma vma;
  bfd_vma lma;
  const char *err;
  const char * name;

  name = bfd_section_name (ibfd, isection);

  osection = bfd_make_section_anyway (obfd, name);

  if (osection == NULL)
    {
      perror("setup_section: making");
    }

  size = bfd_section_size (ibfd, isection);
  if (! bfd_set_section_size (obfd, osection, size))
    {
      perror("setup_section: size");
    }

  vma = bfd_section_vma (ibfd, isection);

  if (! bfd_set_section_vma (obfd, osection, vma))
    {
      perror("setup_section: vma");
    }

  lma = isection->lma;

  osection->lma = lma;

  /* FIXME: This is probably not enough.  If we change the LMA we
     may have to recompute the header for the file as well.  */
  if (!bfd_set_section_alignment (obfd,
				  osection,
				  bfd_section_alignment (ibfd, isection)))
    {
      perror("setup_section: alignment");
    }

  if (!bfd_set_section_flags (obfd, osection, bfd_get_section_flags(ibfd, isection)))
    {
      perror("setup_section: flags");
    }

  /* Copy merge entity size.  */
  osection->entsize = isection->entsize;

  /* This used to be mangle_section; we do here to avoid using
     bfd_get_section_by_name since some formats allow multiple
     sections with the same name.  */

  isection->output_section = osection;
  isection->output_offset = 0;

  /* Allow the BFD backend to copy any private data it understands
     from the input section to the output section.  */
  if (!bfd_copy_private_section_data (ibfd, isection, obfd, osection))
    {
      perror("setup_section: private data");
    }

  /* All went well.  */
  return;

  perror("setup_section: %s: section `%s': error in %s: %s"),
	     bfd_get_filename (ibfd),
	     bfd_section_name (ibfd, isection),
	     err, bfd_errmsg (bfd_get_error ());
}

/* utility function to look up a symbol by its name */
int find_symbol (asymbol **symtab, char *name) {
    int i;
    for (i = 0; symtab[i] != NULL; i++) {
	if (strcmp(bfd_asymbol_name(symtab[i]), name) == 0) {
	    return i;
	}
    }
    return -1;
}

int find_symbol_maxcnt (asymbol **symtab, char *name, int max_idx) {
    int i;
    for (i = 0; (symtab[i] != NULL) && (i < max_idx); i++) {
	if (strcmp(bfd_asymbol_name(symtab[i]), name) == 0) {
	    return i;
	}
    }
    return -1;
}

/* utility function to determine if a particular reloc is one of the ones we'd like to delete */
int is_delete_reloc (const char *name, delete_reloc_struct *del_list_head) {
    delete_reloc_struct *del_ptr;
    for (del_ptr = del_list_head; del_ptr != NULL; del_ptr = del_ptr->next) {
	if (strcmp(name, del_ptr->symbol_name) == 0) {
	    return 1;
	}
    }
    return 0;
}

int change_reloc_idx (char *name, struct change_reloc_struct **changes, int count) {
    int i=0;

    while (i < count) {
	if ((changes[i] != NULL) && (strcmp(name, changes[i]->old_symbol_name) == 0)) {
	    return i;
	}
	i++;
    }
    return -1;
}

/* copy section from ibfd to obfd but make reloc changes as described in global vars */
/* just reloc parts - data comes later */
static void copy_section_relocs_edit (bfd *ibfd, sec_ptr isection, void *obfdarg)
{
    bfd *obfd = obfdarg;
    arelent **relpp;
    long relcount;
    sec_ptr osection;
    bfd_size_type size;
    long relsize;
    flagword flags;


    flags = bfd_get_section_flags (ibfd, isection);
    if ((flags & SEC_GROUP) != 0)
	return;

    osection = isection->output_section;
    size = bfd_get_section_size (isection);

    if (size == 0 || osection == 0)
	return;

    relsize = bfd_get_reloc_upper_bound (ibfd, isection);

    if (relsize <= 0)
    {
	/* not good */
	bfd_perror("attempting to do relocs");
	return;
    }

    relpp = malloc (relsize);

    relcount = bfd_canonicalize_reloc (ibfd, isection, relpp, isympp);

    /* copy each reloc, possibly removing or changing it on the fly */
    arelent **temp_relpp;
    long temp_relcount = 0;
    long i;
    int msg_cnt = 0;
    struct change_reloc_struct *change_ptr;
    struct add_reloc_struct *new_reloc;
    asymbol **osympp;

    /* count new relocs to allocate space in reloc list */
    int reloc_add_cnt;
    reloc_add_cnt = 0;
    for (new_reloc = additional_relocs; new_reloc != NULL; new_reloc = new_reloc->next) {
	reloc_add_cnt++;
    }

    osympp = bfd_get_outsymbols(obfd);

    /* note: cannot run mprobe on osympp b/c the copy contents will sometimes realloc it from internal BFD storage */

    temp_relpp = malloc (relsize + reloc_add_cnt * sizeof(arelent *));
    for (i = 0; i < relcount; i++) {

	if (is_delete_reloc(bfd_asymbol_name(*relpp[i]->sym_ptr_ptr), delete_relocs)) {
	    continue;
	}
	else if ((change_ptr = find_change_reloc(bfd_asymbol_name(*relpp[i]->sym_ptr_ptr), change_relocs)) != NULL) {
	    int sym_idx;
	    sym_idx = find_symbol(osympp, change_ptr->new_symbol_name);
	    if (sym_idx < 0) {
		fprintf(stderr, "internal error: could not find new symbol %s in output symbol table\n", change_ptr->new_symbol_name);
		exit(1);
	    }
	    relpp[i]->sym_ptr_ptr = &(osympp[sym_idx]);
	}
	if ((temp_relcount * sizeof(arelent *)) >= (relsize + reloc_add_cnt * sizeof(arelent *))) {
	    fprintf(stderr, "about to write past end of temp_relpp array! idx=%d, temp_relcount=%d\n", i, temp_relcount);
	}
	temp_relpp [temp_relcount++] = relpp [i];
    }

    /* now the additional relocs from the command line */
    if (strcmp(bfd_get_section_name(ibfd, isection), ".text") == 0) {
	add_reloc_struct *add_reloc;
	for (add_reloc = additional_relocs; add_reloc != NULL; add_reloc = add_reloc->next) {
	    arelent *new_reloc;
	    int symbol_idx;

	    new_reloc = malloc(sizeof(*new_reloc));
	    if ((symbol_idx = find_symbol(osympp, add_reloc->symbol_name)) < 0) {
		fprintf(stderr, "could not find symbol %s to perform a relocation! possible internal error\n", add_reloc->symbol_name);
		continue;
	    }
	    new_reloc->sym_ptr_ptr = &osympp[symbol_idx];
	    new_reloc->address = add_reloc->loc;
	    new_reloc->addend = 0;   /* never seemed to be used in my cursory investigation */
	    new_reloc->howto = bfd_reloc_type_lookup(ibfd, BFD_RELOC_32_PCREL);
	    if (new_reloc->howto == NULL) {
		fprintf(stderr, "could not get howto back from bfd subsystem\n");
		exit(1);
	    }

	    if ((temp_relcount * sizeof(arelent *)) >= (relsize + reloc_add_cnt * sizeof(arelent *))) {
		fprintf(stderr, "about to write past end of temp_relpp array with an add_reloc! temp_relcount=%d\n", temp_relcount);
	    }

	    temp_relpp[temp_relcount++] = new_reloc;
	}
    }
    temp_relpp[temp_relcount] = NULL;

    relcount = temp_relcount;
    free (relpp);
    relpp = temp_relpp;

    bfd_set_reloc (obfd, osection, relcount == 0 ? NULL : relpp, relcount);
    if (relcount == 0)
	free (relpp);

    mcheck_check_all();

}

/* the finishing piece for each section */
static void copy_section_data (bfd *ibfd, sec_ptr isection, void *obfdarg)
{
    bfd *obfd = obfdarg;
    sec_ptr osection;
    bfd_size_type size;
    struct add_reloc_struct *new_reloc;

    size = bfd_get_section_size (isection);

    osection = isection->output_section;

    if (size == 0 || osection == 0)
	return;

    if (bfd_get_section_flags (ibfd, isection) & SEC_HAS_CONTENTS
	&& bfd_get_section_flags (obfd, osection) & SEC_HAS_CONTENTS) {

	void *memhunk;

	mcheck_check_all();

	memhunk = malloc (size);

	if (!bfd_get_section_contents (ibfd, isection, memhunk, 0, size))
	    return;

	if (strcmp(bfd_get_section_name(ibfd, isection), ".text") == 0) {

	    /* make the data modifications, if requested */
	    modify_byte_struct *mbyte_ptr = modify_bytes;
	    while (mbyte_ptr != NULL) {
		((unsigned char *)memhunk)[mbyte_ptr->location] = (char)(mbyte_ptr->value & 0xff);
		mbyte_ptr = mbyte_ptr->next;
	    }

	    /* make sure additional relocs have proper data present */
	    /* kernel implements relocs relative to what's already there in the data! */
	    /* so if we're overriding a call, we need to set the old PC-relative data back to 0xfffffffc */
	    for (new_reloc = additional_relocs; new_reloc != NULL; new_reloc = new_reloc->next) {
		((unsigned char *)memhunk)[new_reloc->loc]   = (unsigned char)0xfc;
		((unsigned char *)memhunk)[new_reloc->loc+1] = (unsigned char)0xff;
		((unsigned char *)memhunk)[new_reloc->loc+2] = (unsigned char)0xff;
		((unsigned char *)memhunk)[new_reloc->loc+3] = (unsigned char)0xff;
	    }
	}

	if (!bfd_set_section_contents (obfd, osection, memhunk, 0, size))
	    return;

	/* note that bfd_set_section_contents will render the symbol table noncanonical */
	/* (the array will not be NULL terminated) */

	free (memhunk);
    }
    else if (!(bfd_get_section_flags (ibfd, isection) & SEC_HAS_CONTENTS)) {
	fprintf(stderr, "not copying input section %s because flags %x do not indicate contents (%x)\n",
	       isection->name,
	       bfd_get_section_flags (ibfd, isection),
	       SEC_HAS_CONTENTS);
    }
    else {
	fprintf(stderr, "not copying output section %s because flags %x do not indicate contents (%x)\n",
	       osection->name,
	       bfd_get_section_flags (obfd, osection),
	       SEC_HAS_CONTENTS);
    }
}

