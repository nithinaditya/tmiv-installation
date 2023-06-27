#!/usr/bin/env python3

# The copyright in this software is being made available under the BSD
# License, included below. This software may be subject to other third party
# and contributor rights, including patent rights, and no such rights are
# granted under this license.
#
# Copyright (c) 2010-2020, ISO/IEC
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#  * Neither the name of the ISO/IEC nor the names of its contributors may
#    be used to endorse or promote products derived from this software without
#    specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
# BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
# THE POSSIBILITY OF SUCH DAMAGE.

import concurrent.futures
import filecmp
import os
import subprocess
import sys
import time


class IntegrationTest:
    def __init__(self, argv):
        if (len(argv) < 5) or (len(argv) % 2 == 0):
            raise RuntimeError(
                'Usage: integration_test.py TMIV_INSTALL_DIR TMIV_SOURCE_DIR CONTENT_DIR TEST_DIR [-g GIT_COMMAND] [-j MAX_WORKERS] [-r REFERENCE_DIR]')

        print('+ {}'.format(' '.join(argv)), flush=True)

        self.tmivInstallDir = os.path.abspath(argv[1])
        self.tmivSourceDir = os.path.abspath(argv[2])
        self.contentDir = os.path.abspath(argv[3])
        self.testDir = os.path.abspath(argv[4])
        self.gitCommand = None
        self.maxWorkers = None
        self.referenceDir = None
        self.numComparisonMismatches = 0
        self.numComparisonErrors = 0

        for i in range(5, len(argv), 2):
            if argv[i] == '-g':
                self.gitCommand = argv[i + 1]
            elif argv[i] == '-j':
                self.maxWorkers = int(argv[i + 1])
            elif argv[i] == '-r':
                self.referenceDir = argv[i + 1]
            else:
                raise RuntimeError('Unknown argument {}'.format(argv[i]))

        self.stop = False

    def run(self):
        print('{{0}} = {}'.format(self.tmivInstallDir))
        print('{{1}} = {}'.format(self.tmivSourceDir))
        print('{{2}} = {}'.format(self.contentDir))
        print('{{3}} = {}'.format(self.testDir), flush=True)

        app.inspectEnvironment()

        with concurrent.futures.ThreadPoolExecutor(max_workers=self.maxWorkers) as executor:
            fA = self.testMivAnchor(executor)
            fV = self.testMivViewAnchor(executor)
            fG = self.testMivDsdeAnchor(executor)
            fR = self.testBestReference(executor)
            fM = self.testMivMpi(executor)
            fS = self.testAdditiveSynthesizer(executor)
            self.sync(fA + fV + fG + fR + fM + fS)

        if self.referenceDir:
            print('Comparison mismatches :', self.numComparisonMismatches)
            print('Comparison errors     :', self.numComparisonErrors)

        return int((0 < self.numComparisonMismatches) or (0 < self.numComparisonErrors))

    def inspectEnvironment(self):
        self.checkDirExists(
            'TMIV installation', self.tmivInstallDir, os.path.join('include', 'TMIV', 'Decoder', 'MivDecoder.h'))
        self.checkDirExists('TMIV source', self.tmivSourceDir,
                            os.path.join('README.md'))
        self.checkDirExists('content', self.contentDir, os.path.join(
            'E', 'v13_texture_1920x1080_yuv420p10le.yuv'))

        os.makedirs(self.testDir, exist_ok=True)

        if self.gitCommand:
            with open(os.path.join(self.testDir, 'git.log'), 'w') as stream:
                for target in [None, stream]:
                    subprocess.run([
                        self.gitCommand, 'log', '-n', '10', '--decorate=short', '--oneline'],
                        shell=False, cwd=self.tmivSourceDir, check=True, stdout=target)
                    subprocess.run(
                        [self.gitCommand, 'status', '--short'],
                        shell=False, cwd=self.tmivSourceDir, check=True, stdout=target)

    def testMivAnchor(self, executor):
        os.makedirs(os.path.join(self.testDir, 'A3/E/QP3'), exist_ok=True)

        f1 = self.launchCommand(executor, [], [
            '{0}/bin/Encoder',
            '-c', '{1}/config/ctc/miv_anchor/A_1_TMIV_encode.json',
            '-p', 'configDirectory', '{1}/config',
            '-p', 'inputDirectory', '{2}',
            '-p', 'outputDirectory', '{3}',
            '-n', '3', '-s', 'E', '-p', 'intraPeriod', '2'],
            '{3}/A3/E/TMIV_A3_E.log',
            ['A3/E/TMIV_A3_E.bit',
             'A3/E/TMIV_A3_E_geo_c00_960x2320_yuv420p10le.yuv',
             'A3/E/TMIV_A3_E_geo_c01_960x2320_yuv420p10le.yuv',
             'A3/E/TMIV_A3_E_tex_c00_1920x4640_yuv420p10le.yuv',
             'A3/E/TMIV_A3_E_tex_c01_1920x4640_yuv420p10le.yuv'])

        f2_1 = self.launchCommand(executor, [f1], [
            '{0}/bin/vvencFFapp',
            '-c', '{1}/config/ctc/miv_anchor/A_2_VVenC_encode_geo.cfg',
            '-i', '{3}/A3/E/TMIV_A3_E_geo_c00_960x2320_yuv420p10le.yuv',
            '-b', '{3}/A3/E/QP3/TMIV_A3_E_QP3_geo_c00.bit',
            '-s', '960x2320', '-q', '20', '-f', '3', '-fr', '30'],
            '{3}/A3/E/QP3/TMIV_A3_E_QP3_geo_c00_vvenc.log',
            ['A3/E/QP3/TMIV_A3_E_QP3_geo_c00.bit'])

        f2_2 = self.launchCommand(executor, [f1], [
            '{0}/bin/vvencFFapp',
            '-c', '{1}/config/ctc/miv_anchor/A_2_VVenC_encode_geo.cfg',
            '-i', '{3}/A3/E/TMIV_A3_E_geo_c01_960x2320_yuv420p10le.yuv',
            '-b', '{3}/A3/E/QP3/TMIV_A3_E_QP3_geo_c01.bit',
            '-s', '960x2320', '-q', '20', '-f', '3', '-fr', '30'],
            '{3}/A3/E/QP3/TMIV_A3_E_QP3_geo_c01_vvenc.log',
            ['A3/E/QP3/TMIV_A3_E_QP3_geo_c01.bit'])

        f2_3 = self.launchCommand(executor, [f1], [
            '{0}/bin/vvencFFapp',
            '-c', '{1}/config/ctc/miv_anchor/A_2_VVenC_encode_tex.cfg',
            '-i', '{3}/A3/E/TMIV_A3_E_tex_c00_1920x4640_yuv420p10le.yuv',
            '-b', '{3}/A3/E/QP3/TMIV_A3_E_QP3_tex_c00.bit',
            '-s', '1920x4640', '-q', '43', '-f', '3', '-fr', '30'],
            '{3}/A3/E/QP3/TMIV_A3_E_QP3_tex_c00_vvenc.log',
            ['A3/E/QP3/TMIV_A3_E_QP3_tex_c00.bit'])

        f2_4 = self.launchCommand(executor, [f1], [
            '{0}/bin/vvencFFapp',
            '-c', '{1}/config/ctc/miv_anchor/A_2_VVenC_encode_tex.cfg',
            '-i', '{3}/A3/E/TMIV_A3_E_tex_c01_1920x4640_yuv420p10le.yuv',
            '-b', '{3}/A3/E/QP3/TMIV_A3_E_QP3_tex_c01.bit',
            '-s', '1920x4640', '-q', '43', '-f', '3', '-fr', '30'],
            '{3}/A3/E/QP3/TMIV_A3_E_QP3_tex_c01_vvenc.log',
            ['A3/E/QP3/TMIV_A3_E_QP3_tex_c01.bit'])

        f2_5 = self.launchCommand(executor, [f1], [
            '{0}/bin/Parser',
            '-b', '{3}/A3/E/TMIV_A3_E.bit'],
            '{3}/A3/E/TMIV_A3_E.hls',
            [])

        f2_6 = self.launchCommand(executor, [f1], [
            '{0}/bin/BitrateReport',
            '-b', '{3}/A3/E/TMIV_A3_E.bit'],
            '{3}/A3/E/TMIV_A3_E.csv',
            [])

        f3_1 = self.launchCommand(executor, [f2_1], [
            '{0}/bin/vvdecapp',
            '-b', '{3}/A3/E/QP3/TMIV_A3_E_QP3_geo_c00.bit',
            '-o', '{3}/A3/E/QP3/TMIV_A3_E_QP3_geo_c00_960x2320_yuv420p10le.yuv'],
            '{3}/A3/E/QP3/TMIV_A3_E_QP3_geo_c00_vvdec.log',
            ['A3/E/QP3/TMIV_A3_E_QP3_geo_c00_960x2320_yuv420p10le.yuv'])

        f3_2 = self.launchCommand(executor, [f2_2], [
            '{0}/bin/vvdecapp',
            '-b', '{3}/A3/E/QP3/TMIV_A3_E_QP3_geo_c01.bit',
            '-o', '{3}/A3/E/QP3/TMIV_A3_E_QP3_geo_c01_960x2320_yuv420p10le.yuv'],
            '{3}/A3/E/QP3/TMIV_A3_E_QP3_geo_c01_vvdec.log',
            ['A3/E/QP3/TMIV_A3_E_QP3_geo_c01_960x2320_yuv420p10le.yuv'])

        f3_3 = self.launchCommand(executor, [f2_3], [
            '{0}/bin/vvdecapp',
            '-b', '{3}/A3/E/QP3/TMIV_A3_E_QP3_tex_c00.bit',
            '-o', '{3}/A3/E/QP3/TMIV_A3_E_QP3_tex_c00_1920x4640_yuv420p10le.yuv'],
            '{3}/A3/E/QP3/TMIV_A3_E_QP3_tex_c00_vvdec.log',
            ['A3/E/QP3/TMIV_A3_E_QP3_tex_c00_1920x4640_yuv420p10le.yuv'])

        f3_4 = self.launchCommand(executor, [f2_4], [
            '{0}/bin/vvdecapp',
            '-b', '{3}/A3/E/QP3/TMIV_A3_E_QP3_tex_c01.bit',
            '-o', '{3}/A3/E/QP3/TMIV_A3_E_QP3_tex_c01_1920x4640_yuv420p10le.yuv'],
            '{3}/A3/E/QP3/TMIV_A3_E_QP3_tex_c01_vvdec.log',
            ['A3/E/QP3/TMIV_A3_E_QP3_tex_c01_1920x4640_yuv420p10le.yuv'])

        f4 = self.launchCommand(executor, [f3_1, f3_2, f3_3, f3_4], [
            '{0}/bin/Decoder',
            '-c', '{1}/config/ctc/miv_anchor/A_4_TMIV_decode.json',
            '-p', 'configDirectory', '{1}/config',
            '-p', 'inputDirectory', '{3}',
            '-p', 'outputDirectory', '{3}',
            '-n', '3', '-N', '3', '-s', 'E', '-r', 'QP3', '-v', 'v11'],
            '{3}/A3/E/QP3/A3_E_QP3_v11.log',
            ['A3/E/QP3/A3_E_QP3_v11_tex_1920x1080_yuv420p10le.yuv'])

        return [f4, f2_5, f2_6]

    def testMivViewAnchor(self, executor):
        os.makedirs(os.path.join(self.testDir, 'V3/D/R0'), exist_ok=True)

        f1 = self.launchCommand(executor, [], [
            '{0}/bin/Encoder',
            '-c', '{1}/config/ctc/miv_view_anchor/V_1_TMIV_encode.json',
            '-p', 'configDirectory', '{1}/config',
            '-p', 'inputDirectory', '{2}',
            '-p', 'outputDirectory', '{3}',
            '-n', '3', '-s', 'D', '-p', 'intraPeriod', '2'],
            '{3}/V3/D/TMIV_V3_D.log',
            ['V3/D/TMIV_V3_D.bit',
             'V3/D/TMIV_V3_D_geo_c00_1024x2176_yuv420p10le.yuv',
             'V3/D/TMIV_V3_D_geo_c01_1024x2176_yuv420p10le.yuv',
             'V3/D/TMIV_V3_D_tex_c00_2048x4352_yuv420p10le.yuv',
             'V3/D/TMIV_V3_D_tex_c01_2048x4352_yuv420p10le.yuv'])

        f2_1 = self.launchCommand(executor, [f1], [
            '{0}/bin/Decoder',
            '-c', '{1}/config/ctc/miv_view_anchor/V_4_TMIV_decode.json',
            '-p', 'configDirectory', '{1}/config',
            '-p', 'inputDirectory', '{3}',
            '-p', 'outputDirectory', '{3}',
            '-p', 'inputGeometryVideoFramePathFmt', 'V{{0}}/{{1}}/TMIV_V{{0}}_{{1}}_geo_c{{3:02}}_{{4}}x{{5}}_yuv420p10le.yuv',
            '-p', 'inputTextureVideoFramePathFmt', 'V{{0}}/{{1}}/TMIV_V{{0}}_{{1}}_tex_c{{3:02}}_{{4}}x{{5}}_yuv420p10le.yuv',
            '-n', '3', '-N', '3', '-s', 'D', '-r', 'R0', '-v', 'v14'],
            '{3}/V3/D/R0/V3_D_R0_v14.log',
            ['V3/D/R0/V3_D_R0_v14_tex_2048x1088_yuv420p10le.yuv'])

        f2_2 = self.launchCommand(executor, [f1], [
            '{0}/bin/Parser',
            '-b', '{3}/V3/D/TMIV_V3_D.bit'],
            '{3}/V3/D/TMIV_V3_D.hls',
            [])

        f2_3 = self.launchCommand(executor, [f1], [
            '{0}/bin/BitrateReport',
            '-b', '{3}/V3/D/TMIV_V3_D.bit'],
            '{3}/V3/D/TMIV_V3_D.csv',
            [])

        f2_4 = self.launchCommand(executor, [f1], [
            '{0}/bin/Decoder',
            '-c', '{1}/config/ctc/miv_view_anchor/V_4_TMIV_decode.json',
            '-p', 'configDirectory', '{1}/config',
            '-p', 'inputDirectory', '{3}',
            '-p', 'outputDirectory', '{3}',
            '-p', 'inputGeometryVideoFramePathFmt', 'V{{0}}/{{1}}/TMIV_V{{0}}_{{1}}_geo_c{{3:02}}_{{4}}x{{5}}_yuv420p10le.yuv',
            '-p', 'inputTextureVideoFramePathFmt', 'V{{0}}/{{1}}/TMIV_V{{0}}_{{1}}_tex_c{{3:02}}_{{4}}x{{5}}_yuv420p10le.yuv',
            '-n', '3', '-N', '3', '-s', 'D', '-r', 'R0', '-P', 'p02'],
            '{3}/V3/D/R0/V3_D_R0_p02.log',
            ['V3/D/R0/V3_D_R0_p02_tex_1920x1080_yuv420p10le.yuv'])

        return [f2_1, f2_2, f2_3, f2_4]

    def testMivDsdeAnchor(self, executor):
        os.makedirs(os.path.join(self.testDir, 'G3/N/R0'), exist_ok=True)

        f1 = self.launchCommand(executor, [], [
            '{0}/bin/Encoder',
            '-c', '{1}/config/ctc/miv_dsde_anchor/G_1_TMIV_encode.json',
            '-p', 'configDirectory', '{1}/config',
            '-p', 'inputDirectory', '{2}',
            '-p', 'outputDirectory', '{3}',
            '-n', '3', '-s', 'N', '-p', 'intraPeriod', '2'],
            '{3}/G3/N/TMIV_G3_N.log',
            ['G3/N/TMIV_G3_N.bit',
             'G3/N/TMIV_G3_N_tex_c00_2048x4352_yuv420p10le.yuv',
             'G3/N/TMIV_G3_N_tex_c01_2048x4352_yuv420p10le.yuv',
             'G3/N/TMIV_G3_N_tex_c02_2048x4352_yuv420p10le.yuv',
             'G3/N/TMIV_G3_N_tex_c03_2048x4352_yuv420p10le.yuv'])

        f2_1 = self.launchCommand(executor, [f1], [
            '{0}/bin/Parser',
            '-b', '{3}/G3/N/TMIV_G3_N.bit'],
            '{3}/G3/N/TMIV_G3_N.hls',
            [])

        f2_2 = self.launchCommand(executor, [f1], [
            '{0}/bin/BitrateReport',
            '-b', '{3}/G3/N/TMIV_G3_N.bit'],
            '{3}/G3/N/TMIV_G3_N.csv',
            [])

        f2_3 = self.launchCommand(executor, [f1], [
            '{0}/bin/Decoder',
            '-c', '{1}/config/ctc/miv_dsde_anchor/G_4_TMIV_decode.json',
            '-p', 'configDirectory', '{1}/config',
            '-p', 'inputDirectory', '{3}',
            '-p', 'outputDirectory', '{3}',
            '-p', 'inputGeometryVideoFramePathFmt', 'G{{0}}/{{1}}/TMIV_G{{0}}_{{1}}_geo_c{{3:02}}_{{4}}x{{5}}_yuv420p10le.yuv',
            '-p', 'inputTextureVideoFramePathFmt', 'G{{0}}/{{1}}/TMIV_G{{0}}_{{1}}_tex_c{{3:02}}_{{4}}x{{5}}_yuv420p10le.yuv',
            '-n', '3', '-N', '3', '-s', 'N', '-r', 'R0'],
            '{3}/G3/N/R0/G3_N_R0_none.log',
            ['G3/N/R0/TMIV_G3_N_R0_0000.json',
             'G3/N/R0/TMIV_G3_N_R0_tex_pv00_2048x2048_yuv420p10le.yuv',
             'G3/N/R0/TMIV_G3_N_R0_tex_pv01_2048x2048_yuv420p10le.yuv',
             'G3/N/R0/TMIV_G3_N_R0_tex_pv02_2048x2048_yuv420p10le.yuv',
             'G3/N/R0/TMIV_G3_N_R0_tex_pv03_2048x2048_yuv420p10le.yuv',
             'G3/N/R0/TMIV_G3_N_R0_tex_pv04_2048x2048_yuv420p10le.yuv',
             'G3/N/R0/TMIV_G3_N_R0_tex_pv05_2048x2048_yuv420p10le.yuv',
             'G3/N/R0/TMIV_G3_N_R0_tex_pv06_2048x2048_yuv420p10le.yuv'])

        return [f2_1, f2_2, f2_3]

    def testBestReference(self, executor):
        os.makedirs(os.path.join(self.testDir, 'R3/O/R0'), exist_ok=True)

        f1_1 = self.launchCommand(executor, [], [
            '{0}/bin/Renderer',
            '-c', '{1}/config/ctc/best_reference/R_1_TMIV_render.json',
            '-p', 'configDirectory', '{1}/config',
            '-p', 'inputDirectory', '{2}',
            '-p', 'outputDirectory', '{3}',
            '-n', '3', '-N', '3', '-s', 'O', '-r', 'R0', '-v', 'v01'],
            '{3}/R3/O/R0/R3_O_R0_v01.log',
            ['R3/O/R0/R3_O_R0_v01_geo_1920x1080_yuv420p16le.yuv',
             'R3/O/R0/R3_O_R0_v01_tex_1920x1080_yuv420p10le.yuv'])

        f1_2 = self.launchCommand(executor, [], [
            '{0}/bin/Renderer',
            '-c', '{1}/config/ctc/best_reference/R_1_TMIV_render.json',
            '-p', 'configDirectory', '{1}/config',
            '-p', 'inputDirectory', '{2}',
            '-p', 'outputDirectory', '{3}',
            '-n', '3', '-N', '3', '-s', 'O', '-r', 'R0', '-P', 'p02'],
            '{3}/R3/O/R0/R3_O_R0_p02.log',
            ['R3/O/R0/R3_O_R0_p02_geo_1920x1080_yuv420p16le.yuv',
             'R3/O/R0/R3_O_R0_p02_tex_1920x1080_yuv420p10le.yuv'])

        return [f1_1, f1_2]

    def testMivMpi(self, executor):
        os.makedirs(os.path.join(self.testDir, 'M3/M/QP3'), exist_ok=True)

        f1 = self.launchCommand(executor, [], [
            '{0}/bin/MpiEncoder',
            '-c', '{1}/config/test/miv_mpi/M_1_TMIV_encode.json',
            '-p', 'configDirectory', '{1}/config',
            '-p', 'inputDirectory', '{2}',
            '-p', 'outputDirectory', '{3}',
            '-n', '3', '-s', 'M', '-p', 'intraPeriod', '2'],
            '{3}/M3/M/TMIV_M3_M.log',
            ['M3/M/TMIV_M3_M.bit',
             'M3/M/TMIV_M3_M_tra_c00_4096x4096_yuv420p10le.yuv',
             'M3/M/TMIV_M3_M_tex_c00_4096x4096_yuv420p10le.yuv'])

        f2_1 = self.launchCommand(executor, [f1], [
            '{0}/bin/TAppEncoder',
            '-c', '{1}/config/test/miv_mpi/M_2_HM_encode_tra.cfg',
            '-i', '{3}/M3/M/TMIV_M3_M_tra_c00_4096x4096_yuv420p10le.yuv',
            '-b', '{3}/M3/M/QP3/TMIV_M3_M_QP3_tra_c00.bit',
            '-wdt', '4096', '-hgt', '4096', '-q', '5', '-f', '3', '-fr', '30'],
            '{3}/M3/M/QP3/TMIV_M3_M_QP3_tra_c00.log',
            ['M3/M/QP3/TMIV_M3_M_QP3_tra_c00.bit'])

        f2_2 = self.launchCommand(executor, [f1], [
            '{0}/bin/TAppEncoder',
            '-c', '{1}/config/test/miv_mpi/M_2_HM_encode_tex.cfg',
            '-i', '{3}/M3/M/TMIV_M3_M_tex_c00_4096x4096_yuv420p10le.yuv',
            '-b', '{3}/M3/M/QP3/TMIV_M3_M_QP3_tex_c00.bit',
            '-wdt', '4096', '-hgt', '4096', '-q', '30', '-f', '3', '-fr', '30'],
            '{3}/M3/M/QP3/TMIV_M3_M_QP3_tex_c00.log',
            ['M3/M/QP3/TMIV_M3_M_QP3_tex_c00.bit'])

        f3 = self.launchCommand(executor, [f2_1, f2_2], [
            '{0}/bin/Multiplexer',
            '-c', '{1}/config/test/miv_mpi/M_3_TMIV_mux.json',
            '-p', 'configDirectory', '{1}/config',
            '-p', 'inputDirectory', '{3}',
            '-p', 'outputDirectory', '{3}',
            '-n', '3', '-s', 'M', '-r', 'QP3'],
            '{3}/M3/M/QP3/TMIV_M3_M_QP3.log',
            ['M3/M/QP3/TMIV_M3_M_QP3.bit'])

        f4_1 = self.launchCommand(executor, [f3], [
            '{0}/bin/Parser',
            '-b', '{3}/M3/M/QP3/TMIV_M3_M_QP3.bit'],
            '{3}/M3/M/QP3/TMIV_M3_M_QP3.hls',
            [])

        f4_2 = self.launchCommand(executor, [f3], [
            '{0}/bin/BitrateReport',
            '-b', '{3}/M3/M/QP3/TMIV_M3_M_QP3.bit'],
            '{3}/M3/M/QP3/TMIV_M3_M_QP3.csv',
            [])

        f4_3 = self.launchCommand(executor, [f3], [
            '{0}/bin/Decoder',
            '-c', '{1}/config/test/miv_mpi/M_4_TMIV_decode.json',
            '-p', 'configDirectory', '{1}/config',
                  '-p', 'inputDirectory', '{3}',
                  '-p', 'outputDirectory', '{3}',
            '-n', '3', '-N', '3', '-s', 'M', '-r', 'QP3', '-v', 'viewport'],
            '{3}/M3/M/QP3/M3_M_QP3_viewport.log',
            ['M3/M/QP3/M3_M_QP3_viewport_tex_1920x1080_yuv420p10le.yuv'])

        return [f4_1, f4_2, f4_3]

    def testAdditiveSynthesizer(self, executor):
        os.makedirs(os.path.join(self.testDir, 'S1/C/R0'), exist_ok=True)

        f1 = self.launchCommand(executor, [], [
            '{0}/bin/Renderer',
            '-c', '{1}/config/test/additive_synthesizer/S_1_TMIV_render.json',
            '-p', 'configDirectory', '{1}/config',
            '-p', 'inputDirectory', '{2}',
            '-p', 'outputDirectory', '{3}',
            '-n', '1', '-N', '1', '-s', 'C', '-r', 'R0', '-P', 'p03'],
            '{3}/S1/C/R0/S1_C_R0_p03.log',
            ['S1/C/R0/S1_C_R0_p03_geo_2048x2048_yuv420p16le.yuv',
             'S1/C/R0/S1_C_R0_p03_tex_2048x2048_yuv420p10le.yuv'])

        return [f1]

    def launchCommand(self, executor, futures, args, logFile, outputFiles):
        return executor.submit(self.syncAndRunCommand, futures, args, logFile, outputFiles)

    def syncAndRunCommand(self, futures, args, logFile, outputFiles):
        for future in concurrent.futures.as_completed(futures):
            future.result()

        self.runCommand(args, logFile)

        if self.referenceDir:
            self.compareFiles(outputFiles)

    def sync(self, futures):
        for future in concurrent.futures.as_completed(futures):
            try:
                future.result()
            except Exception as exception:
                self.stop = True
                raise exception

    def runCommand(self, args, logFile):
        # Print the command itself
        print('+ {}'.format(' '.join(args)), flush=True)

        # Replace placeholders within arguments
        def f(arg): return arg.format(self.tmivInstallDir,
                                      self.tmivSourceDir, self.contentDir, self.testDir)
        args = list(map(f, args))
        logFile = f(logFile)

        # Execute the command in an interuptable way
        popen = subprocess.Popen(args, shell=False, cwd=self.testDir, stdout=open(
            logFile, 'w'), stderr=subprocess.STDOUT)

        while True:
            time.sleep(1)
            returncode = popen.poll()
            if self.stop:
                print('Killing process {}'.format(args[0]))
                popen.kill()
                sys.exit(1)
            elif returncode is None:
                continue
            elif returncode == 0:
                return
            else:
                raise RuntimeError(
                    'EXECUTION FAILED!\n  * Log-file    : {}\n  * Command     : {}'.format(logFile, ' '.join(args)))

    def checkDirExists(self, what, dir, probe):
        if not os.path.isdir(dir):
            raise RuntimeError('Directory {} does not exist.'.format(dir))

        if not os.path.exists(os.path.join(dir, probe)):
            raise RuntimeError(
                '{} does not appear to be a {} directory because {} was not found.'.format(dir, what, probe))

    def compareFiles(self, outputFiles):
        assert(self.referenceDir)
        [matches, mismatches, errors] = filecmp.cmpfiles(
            self.testDir, self.referenceDir, outputFiles, shallow=False)

        for match in matches:
            print('equal: {}'.format(match), flush=True)
        for mismatch in mismatches:
            print('MISMATCH: {}'.format(mismatch), flush=True)
        for error in errors:
            print('ERROR: {}'.format(error), flush=True)

        self.numComparisonMismatches += len(mismatches)
        self.numComparisonErrors += len(errors)


if __name__ == '__main__':
    try:
        app = IntegrationTest(sys.argv)
        exit(app.run())
    except RuntimeError as e:
        print(e)
        exit(1)
