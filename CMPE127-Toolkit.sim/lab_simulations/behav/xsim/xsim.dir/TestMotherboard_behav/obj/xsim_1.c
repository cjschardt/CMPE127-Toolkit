/**********************************************************************/
/*   ____  ____                                                       */
/*  /   /\/   /                                                       */
/* /___/  \  /                                                        */
/* \   \   \/                                                         */
/*  \   \        Copyright (c) 2003-2013 Xilinx, Inc.                 */
/*  /   /        All Right Reserved.                                  */
/* /---/   /\                                                         */
/* \   \  /  \                                                        */
/*  \___\/\___\                                                       */
/**********************************************************************/


#include "iki.h"
#include <string.h>
#include <math.h>
#ifdef __GNUC__
#include <stdlib.h>
#else
#include <malloc.h>
#define alloca _alloca
#endif
/**********************************************************************/
/*   ____  ____                                                       */
/*  /   /\/   /                                                       */
/* /___/  \  /                                                        */
/* \   \   \/                                                         */
/*  \   \        Copyright (c) 2003-2013 Xilinx, Inc.                 */
/*  /   /        All Right Reserved.                                  */
/* /---/   /\                                                         */
/* \   \  /  \                                                        */
/*  \___\/\___\                                                       */
/**********************************************************************/


#include "iki.h"
#include <string.h>
#include <math.h>
#ifdef __GNUC__
#include <stdlib.h>
#else
#include <malloc.h>
#define alloca _alloca
#endif
typedef void (*funcp)(char *, char *);
extern void execute_110(char*, char *);
extern void execute_271(char*, char *);
extern void execute_272(char*, char *);
extern void execute_273(char*, char *);
extern void execute_274(char*, char *);
extern void execute_115(char*, char *);
extern void execute_116(char*, char *);
extern void execute_247(char*, char *);
extern void execute_248(char*, char *);
extern void vlog_const_rhs_process_execute_0_fast_no_reg_no_agg(char*, char*, char*);
extern void execute_250(char*, char *);
extern void execute_251(char*, char *);
extern void execute_252(char*, char *);
extern void execute_253(char*, char *);
extern void execute_254(char*, char *);
extern void execute_255(char*, char *);
extern void execute_256(char*, char *);
extern void execute_257(char*, char *);
extern void execute_258(char*, char *);
extern void execute_259(char*, char *);
extern void execute_260(char*, char *);
extern void execute_261(char*, char *);
extern void execute_262(char*, char *);
extern void execute_263(char*, char *);
extern void execute_264(char*, char *);
extern void execute_265(char*, char *);
extern void execute_266(char*, char *);
extern void execute_267(char*, char *);
extern void execute_268(char*, char *);
extern void execute_269(char*, char *);
extern void execute_270(char*, char *);
extern void execute_117(char*, char *);
extern void execute_118(char*, char *);
extern void execute_121(char*, char *);
extern void execute_122(char*, char *);
extern void execute_6(char*, char *);
extern void execute_119(char*, char *);
extern void execute_8(char*, char *);
extern void execute_120(char*, char *);
extern void execute_158(char*, char *);
extern void execute_159(char*, char *);
extern void execute_160(char*, char *);
extern void execute_161(char*, char *);
extern void execute_162(char*, char *);
extern void execute_127(char*, char *);
extern void execute_128(char*, char *);
extern void execute_123(char*, char *);
extern void execute_124(char*, char *);
extern void execute_16(char*, char *);
extern void execute_18(char*, char *);
extern void execute_129(char*, char *);
extern void execute_130(char*, char *);
extern void execute_131(char*, char *);
extern void execute_21(char*, char *);
extern void execute_22(char*, char *);
extern void execute_132(char*, char *);
extern void execute_133(char*, char *);
extern void execute_134(char*, char *);
extern void execute_135(char*, char *);
extern void execute_136(char*, char *);
extern void execute_137(char*, char *);
extern void execute_28(char*, char *);
extern void execute_152(char*, char *);
extern void execute_153(char*, char *);
extern void execute_154(char*, char *);
extern void execute_155(char*, char *);
extern void execute_156(char*, char *);
extern void execute_32(char*, char *);
extern void execute_140(char*, char *);
extern void execute_141(char*, char *);
extern void execute_142(char*, char *);
extern void execute_143(char*, char *);
extern void execute_144(char*, char *);
extern void execute_145(char*, char *);
extern void execute_146(char*, char *);
extern void execute_147(char*, char *);
extern void execute_38(char*, char *);
extern void execute_150(char*, char *);
extern void execute_43(char*, char *);
extern void execute_45(char*, char *);
extern void execute_48(char*, char *);
extern void execute_50(char*, char *);
extern void execute_53(char*, char *);
extern void execute_163(char*, char *);
extern void execute_164(char*, char *);
extern void execute_165(char*, char *);
extern void execute_167(char*, char *);
extern void execute_77(char*, char *);
extern void execute_169(char*, char *);
extern void execute_170(char*, char *);
extern void execute_171(char*, char *);
extern void execute_193(char*, char *);
extern void execute_194(char*, char *);
extern void execute_195(char*, char *);
extern void execute_196(char*, char *);
extern void execute_197(char*, char *);
extern void execute_198(char*, char *);
extern void execute_199(char*, char *);
extern void execute_200(char*, char *);
extern void execute_172(char*, char *);
extern void execute_173(char*, char *);
extern void vlog_simple_process_execute_0_fast_no_reg_no_agg(char*, char*, char*);
extern void execute_179(char*, char *);
extern void execute_180(char*, char *);
extern void execute_181(char*, char *);
extern void execute_182(char*, char *);
extern void execute_183(char*, char *);
extern void execute_186(char*, char *);
extern void execute_187(char*, char *);
extern void execute_188(char*, char *);
extern void execute_189(char*, char *);
extern void execute_190(char*, char *);
extern void execute_191(char*, char *);
extern void execute_62(char*, char *);
extern void execute_66(char*, char *);
extern void execute_68(char*, char *);
extern void execute_175(char*, char *);
extern void execute_176(char*, char *);
extern void execute_178(char*, char *);
extern void execute_74(char*, char *);
extern void execute_76(char*, char *);
extern void execute_201(char*, char *);
extern void execute_80(char*, char *);
extern void execute_81(char*, char *);
extern void execute_202(char*, char *);
extern void execute_83(char*, char *);
extern void execute_85(char*, char *);
extern void execute_203(char*, char *);
extern void execute_205(char*, char *);
extern void execute_206(char*, char *);
extern void execute_229(char*, char *);
extern void execute_230(char*, char *);
extern void execute_231(char*, char *);
extern void execute_232(char*, char *);
extern void execute_233(char*, char *);
extern void execute_234(char*, char *);
extern void execute_235(char*, char *);
extern void execute_236(char*, char *);
extern void execute_237(char*, char *);
extern void execute_238(char*, char *);
extern void execute_239(char*, char *);
extern void execute_240(char*, char *);
extern void execute_241(char*, char *);
extern void execute_242(char*, char *);
extern void execute_243(char*, char *);
extern void execute_244(char*, char *);
extern void execute_245(char*, char *);
extern void execute_246(char*, char *);
extern void execute_89(char*, char *);
extern void execute_91(char*, char *);
extern void execute_207(char*, char *);
extern void execute_208(char*, char *);
extern void execute_209(char*, char *);
extern void execute_210(char*, char *);
extern void execute_211(char*, char *);
extern void execute_212(char*, char *);
extern void execute_213(char*, char *);
extern void execute_214(char*, char *);
extern void execute_93(char*, char *);
extern void execute_215(char*, char *);
extern void execute_216(char*, char *);
extern void execute_217(char*, char *);
extern void execute_218(char*, char *);
extern void execute_219(char*, char *);
extern void execute_97(char*, char *);
extern void execute_98(char*, char *);
extern void execute_220(char*, char *);
extern void execute_222(char*, char *);
extern void execute_106(char*, char *);
extern void execute_225(char*, char *);
extern void execute_226(char*, char *);
extern void execute_227(char*, char *);
extern void execute_228(char*, char *);
extern void execute_112(char*, char *);
extern void execute_113(char*, char *);
extern void execute_114(char*, char *);
extern void execute_275(char*, char *);
extern void execute_276(char*, char *);
extern void execute_277(char*, char *);
extern void execute_278(char*, char *);
extern void execute_279(char*, char *);
extern void vlog_transfunc_eventcallback(char*, char*, unsigned, unsigned, unsigned, char *);
extern void transaction_16(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_17(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_19(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_36(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_56(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_57(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_58(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_60(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_61(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_62(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_63(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_64(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_65(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_66(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_67(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_68(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_69(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_71(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_72(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_73(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_74(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_80(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_81(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_82(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_83(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_84(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_85(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_86(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_87(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_88(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_89(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_90(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_91(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_92(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_93(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_94(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_95(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_96(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_97(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_99(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_100(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_101(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_102(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_103(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_104(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_105(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_106(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_108(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_109(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_110(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_112(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_113(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_114(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_115(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_116(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_117(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_118(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_119(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_120(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_188(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_191(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_192(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_193(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_242(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_244(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_250(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_251(char*, char*, unsigned, unsigned, unsigned);
extern void transaction_258(char*, char*, unsigned, unsigned, unsigned);
funcp funcTab[250] = {(funcp)execute_110, (funcp)execute_271, (funcp)execute_272, (funcp)execute_273, (funcp)execute_274, (funcp)execute_115, (funcp)execute_116, (funcp)execute_247, (funcp)execute_248, (funcp)vlog_const_rhs_process_execute_0_fast_no_reg_no_agg, (funcp)execute_250, (funcp)execute_251, (funcp)execute_252, (funcp)execute_253, (funcp)execute_254, (funcp)execute_255, (funcp)execute_256, (funcp)execute_257, (funcp)execute_258, (funcp)execute_259, (funcp)execute_260, (funcp)execute_261, (funcp)execute_262, (funcp)execute_263, (funcp)execute_264, (funcp)execute_265, (funcp)execute_266, (funcp)execute_267, (funcp)execute_268, (funcp)execute_269, (funcp)execute_270, (funcp)execute_117, (funcp)execute_118, (funcp)execute_121, (funcp)execute_122, (funcp)execute_6, (funcp)execute_119, (funcp)execute_8, (funcp)execute_120, (funcp)execute_158, (funcp)execute_159, (funcp)execute_160, (funcp)execute_161, (funcp)execute_162, (funcp)execute_127, (funcp)execute_128, (funcp)execute_123, (funcp)execute_124, (funcp)execute_16, (funcp)execute_18, (funcp)execute_129, (funcp)execute_130, (funcp)execute_131, (funcp)execute_21, (funcp)execute_22, (funcp)execute_132, (funcp)execute_133, (funcp)execute_134, (funcp)execute_135, (funcp)execute_136, (funcp)execute_137, (funcp)execute_28, (funcp)execute_152, (funcp)execute_153, (funcp)execute_154, (funcp)execute_155, (funcp)execute_156, (funcp)execute_32, (funcp)execute_140, (funcp)execute_141, (funcp)execute_142, (funcp)execute_143, (funcp)execute_144, (funcp)execute_145, (funcp)execute_146, (funcp)execute_147, (funcp)execute_38, (funcp)execute_150, (funcp)execute_43, (funcp)execute_45, (funcp)execute_48, (funcp)execute_50, (funcp)execute_53, (funcp)execute_163, (funcp)execute_164, (funcp)execute_165, (funcp)execute_167, (funcp)execute_77, (funcp)execute_169, (funcp)execute_170, (funcp)execute_171, (funcp)execute_193, (funcp)execute_194, (funcp)execute_195, (funcp)execute_196, (funcp)execute_197, (funcp)execute_198, (funcp)execute_199, (funcp)execute_200, (funcp)execute_172, (funcp)execute_173, (funcp)vlog_simple_process_execute_0_fast_no_reg_no_agg, (funcp)execute_179, (funcp)execute_180, (funcp)execute_181, (funcp)execute_182, (funcp)execute_183, (funcp)execute_186, (funcp)execute_187, (funcp)execute_188, (funcp)execute_189, (funcp)execute_190, (funcp)execute_191, (funcp)execute_62, (funcp)execute_66, (funcp)execute_68, (funcp)execute_175, (funcp)execute_176, (funcp)execute_178, (funcp)execute_74, (funcp)execute_76, (funcp)execute_201, (funcp)execute_80, (funcp)execute_81, (funcp)execute_202, (funcp)execute_83, (funcp)execute_85, (funcp)execute_203, (funcp)execute_205, (funcp)execute_206, (funcp)execute_229, (funcp)execute_230, (funcp)execute_231, (funcp)execute_232, (funcp)execute_233, (funcp)execute_234, (funcp)execute_235, (funcp)execute_236, (funcp)execute_237, (funcp)execute_238, (funcp)execute_239, (funcp)execute_240, (funcp)execute_241, (funcp)execute_242, (funcp)execute_243, (funcp)execute_244, (funcp)execute_245, (funcp)execute_246, (funcp)execute_89, (funcp)execute_91, (funcp)execute_207, (funcp)execute_208, (funcp)execute_209, (funcp)execute_210, (funcp)execute_211, (funcp)execute_212, (funcp)execute_213, (funcp)execute_214, (funcp)execute_93, (funcp)execute_215, (funcp)execute_216, (funcp)execute_217, (funcp)execute_218, (funcp)execute_219, (funcp)execute_97, (funcp)execute_98, (funcp)execute_220, (funcp)execute_222, (funcp)execute_106, (funcp)execute_225, (funcp)execute_226, (funcp)execute_227, (funcp)execute_228, (funcp)execute_112, (funcp)execute_113, (funcp)execute_114, (funcp)execute_275, (funcp)execute_276, (funcp)execute_277, (funcp)execute_278, (funcp)execute_279, (funcp)vlog_transfunc_eventcallback, (funcp)transaction_16, (funcp)transaction_17, (funcp)transaction_19, (funcp)transaction_36, (funcp)transaction_56, (funcp)transaction_57, (funcp)transaction_58, (funcp)transaction_60, (funcp)transaction_61, (funcp)transaction_62, (funcp)transaction_63, (funcp)transaction_64, (funcp)transaction_65, (funcp)transaction_66, (funcp)transaction_67, (funcp)transaction_68, (funcp)transaction_69, (funcp)transaction_71, (funcp)transaction_72, (funcp)transaction_73, (funcp)transaction_74, (funcp)transaction_80, (funcp)transaction_81, (funcp)transaction_82, (funcp)transaction_83, (funcp)transaction_84, (funcp)transaction_85, (funcp)transaction_86, (funcp)transaction_87, (funcp)transaction_88, (funcp)transaction_89, (funcp)transaction_90, (funcp)transaction_91, (funcp)transaction_92, (funcp)transaction_93, (funcp)transaction_94, (funcp)transaction_95, (funcp)transaction_96, (funcp)transaction_97, (funcp)transaction_99, (funcp)transaction_100, (funcp)transaction_101, (funcp)transaction_102, (funcp)transaction_103, (funcp)transaction_104, (funcp)transaction_105, (funcp)transaction_106, (funcp)transaction_108, (funcp)transaction_109, (funcp)transaction_110, (funcp)transaction_112, (funcp)transaction_113, (funcp)transaction_114, (funcp)transaction_115, (funcp)transaction_116, (funcp)transaction_117, (funcp)transaction_118, (funcp)transaction_119, (funcp)transaction_120, (funcp)transaction_188, (funcp)transaction_191, (funcp)transaction_192, (funcp)transaction_193, (funcp)transaction_242, (funcp)transaction_244, (funcp)transaction_250, (funcp)transaction_251, (funcp)transaction_258};
const int NumRelocateId= 250;

void relocate(char *dp)
{
	iki_relocate(dp, "xsim.dir/TestMotherboard_behav/xsim.reloc",  (void **)funcTab, 250);

	/*Populate the transaction function pointer field in the whole net structure */
}

void sensitize(char *dp)
{
	iki_sensitize(dp, "xsim.dir/TestMotherboard_behav/xsim.reloc");
}

void simulate(char *dp)
{
	iki_schedule_processes_at_time_zero(dp, "xsim.dir/TestMotherboard_behav/xsim.reloc");
	// Initialize Verilog nets in mixed simulation, for the cases when the value at time 0 should be propagated from the mixed language Vhdl net
	iki_execute_processes();

	// Schedule resolution functions for the multiply driven Verilog nets that have strength
	// Schedule transaction functions for the singly driven Verilog nets that have strength

}
#include "iki_bridge.h"
void relocate(char *);

void sensitize(char *);

void simulate(char *);

int main(int argc, char **argv)
{
    iki_heap_initialize("ms", "isimmm", 0, 10485760) ;
    iki_set_sv_type_file_path_name("xsim.dir/TestMotherboard_behav/xsim.svtype");
    iki_set_crvs_dump_file_path_name("xsim.dir/TestMotherboard_behav/xsim.crvsdump");
    void* design_handle = iki_create_design("xsim.dir/TestMotherboard_behav/xsim.mem", (void *)relocate, (void *)sensitize, (void *)simulate, 0, isimBridge_getWdbWriter(), 0, argc, argv);
     iki_set_rc_trial_count(100);
    (void) design_handle;
    return iki_simulate_design();
}
