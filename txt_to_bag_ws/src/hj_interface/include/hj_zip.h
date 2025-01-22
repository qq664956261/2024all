/**
 * @file hj_zip.h
 * @author hao wu (clayderman@yardbot.net)
 * @brief 
 * @version 0.1
 * @date 2024-07-15
 * 
 * @copyright Copyright 2023 HJ Technology Co.Ltd. All rights reserved
 * Copyright (C) 1999-2020 Dieter Baron and Thomas Klausner

*The authors can be contacted at <info@libzip.org>
*Redistribution and use in source and binary forms, with or without
*modification, are permitted provided that the following conditions
*are met:
*
*1. Redistributions of source code must retain the above copyright
*  notice, this list of conditions and the following disclaimer.
*
*2. Redistributions in binary form must reproduce the above copyright
*  notice, this list of conditions and the following disclaimer in
*  the documentation and/or other materials provided with the
*  distribution.
*
*3. The names of the authors may not be used to endorse or promote
*  products derived from this software without specific prior
*  written permission.

*THIS SOFTWARE IS PROVIDED BY THE AUTHORS ``AS IS'' AND ANY EXPRESS
*OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
*WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
*ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY
*DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
*DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
*GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
*INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
*IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
*OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
*IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#ifndef SRC_BASE_FRAMEWORK_INCLUDE_HJ_ZIP_H_
#define SRC_BASE_FRAMEWORK_INCLUDE_HJ_ZIP_H_
#include <vector>
#include <string>
#include "zipconf.h"
#include "zip.h"
#define ZIP_FILE_START_OFFSET 0


namespace hj_bf {
  /**
   * @brief Create a Zip File By Dir object
   * 
   * @param zip_file_name 
   * @param dir_path 
   * @return true 
   * @return false 
   */
  bool CreateZipFileByDir(const std::string& zip_file_name, const std::string& dir_path);
  /**
   * @brief Create a Zip File By File object
   * 
   * @param zip_file_name 
   * @param file_path 
   * @return true 
   * @return false 
   */
  bool CreateZipFileByFile(const std::string& zip_file_name, const std::string& file_path);
  /**
   * @brief Create a Zip File By Files object
   * 
   * @param zip_file_name 
   * @param file_lists 
   * @return true 
   * @return false 
   */
  bool CreateZipFileByFiles(const std::string& zip_file_name, const std::vector<std::string>& file_lists);

}  // namespace hj_bf

#endif  // SRC_BASE_FRAMEWORK_INCLUDE_HJ_ZIP_H_
