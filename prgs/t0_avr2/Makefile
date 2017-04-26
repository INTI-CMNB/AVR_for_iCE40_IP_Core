OBJDIR=../../Work
PRJS=t0_avr2 tldi_out tadd_1 tsub_1 tlogic_1 timm_1 tcom_1 tneg_1 tinc_1 tlsr_1 \
	tror_1 tasr_1 tswap_mov_1 tadiw_1 tsbiw_1 tlds_sts_1 tld_st_1 tld_st_2 \
	trcall_1 ticall_1 trjmp_1 tijmp_1 tbrbx_bset_1 tsbix_sbi_1 tsbrx_1 tcp_1 \
	tbst_bld_1 tpush_pop tlpm_1 tirq_1 tlds_sts_2 trjmp_2 tlpm_spm_1
HEXS=$(addsuffix .hex,$(addprefix $(OBJDIR)/,$(PRJS)))

all: $(HEXS)

$(OBJDIR)/%.hex: %.asm
	cd $(OBJDIR); ln -fs ../prgs/t0_avr2/$< $<
	avra -l $(OBJDIR)/$<.lst $(OBJDIR)/$<

