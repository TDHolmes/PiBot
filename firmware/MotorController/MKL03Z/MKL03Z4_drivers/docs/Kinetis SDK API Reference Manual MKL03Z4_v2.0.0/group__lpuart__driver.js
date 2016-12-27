var group__lpuart__driver =
[
    [ "lpuart_config_t", "group__lpuart__driver.html#structlpuart__config__t", [
      [ "baudRate_Bps", "group__lpuart__driver.html#a09d359de28dc114424b2d702df014d1c", null ],
      [ "parityMode", "group__lpuart__driver.html#ae7ce86796f025a059c973c540d5a94ad", null ],
      [ "dataBitsCount", "group__lpuart__driver.html#a58ce25b71bde98b9c09875481637d120", null ],
      [ "isMsb", "group__lpuart__driver.html#aae9f69c98294d67da66edbce283f029b", null ],
      [ "enableTx", "group__lpuart__driver.html#a0d677467cd14ee90f544d688f3dc9b9a", null ],
      [ "enableRx", "group__lpuart__driver.html#a48d3f37a9364c0093083ef843d5d062e", null ]
    ] ],
    [ "lpuart_transfer_t", "group__lpuart__driver.html#structlpuart__transfer__t", [
      [ "data", "group__lpuart__driver.html#a7c49cf389dea8ad6f674bff6cedd8e37", null ],
      [ "dataSize", "group__lpuart__driver.html#ad269a833a8e8e4cdbc0d485d59e256d8", null ]
    ] ],
    [ "lpuart_handle_t", "group__lpuart__driver.html#struct__lpuart__handle", [
      [ "txData", "group__lpuart__driver.html#a85dfd2cc5aa30f259ce7338a48c832a8", null ],
      [ "txDataSize", "group__lpuart__driver.html#a2114edec74578fe798a62cf852ab0194", null ],
      [ "txDataSizeAll", "group__lpuart__driver.html#a84153c9f581e7201d5c2a20423a321da", null ],
      [ "rxData", "group__lpuart__driver.html#a6d3fceca5b3bee7f183116ee7b3a3b93", null ],
      [ "rxDataSize", "group__lpuart__driver.html#a6b9d688ed6ecbb3b71d6b266bec66edb", null ],
      [ "rxDataSizeAll", "group__lpuart__driver.html#a4ac2cb3c691238ed65a34410ab8f2920", null ],
      [ "rxRingBuffer", "group__lpuart__driver.html#a3eca0396fcb49d3652a40c7cf49024c8", null ],
      [ "rxRingBufferSize", "group__lpuart__driver.html#a12cff540159d2fb592e4856957d820cd", null ],
      [ "rxRingBufferHead", "group__lpuart__driver.html#aaffed28a2a686bb90e3238a3d8e597b3", null ],
      [ "rxRingBufferTail", "group__lpuart__driver.html#acecaaa5df8327c5b33815cb7483abd75", null ],
      [ "callback", "group__lpuart__driver.html#a881c1d7231d1ee0bcb3eb5f1447f75a4", null ],
      [ "userData", "group__lpuart__driver.html#ad9f3989cffe26d3ca63d381da36024be", null ],
      [ "txState", "group__lpuart__driver.html#a86fe3c8f354afbe67d28a71068206b26", null ],
      [ "rxState", "group__lpuart__driver.html#a088a1e9a009d852b90fb8390fa7cbbcd", null ]
    ] ],
    [ "FSL_LPUART_DRIVER_VERSION", "group__lpuart__driver.html#ga0870fb824ece32739bd35f819f8c408e", null ],
    [ "lpuart_transfer_callback_t", "group__lpuart__driver.html#ga7ab1637091d166aa8b69517350fb05c8", null ],
    [ "_lpuart_status", "group__lpuart__driver.html#ga91d929761e975dda91c0cc811d253ce5", [
      [ "kStatus_LPUART_TxBusy", "group__lpuart__driver.html#gga91d929761e975dda91c0cc811d253ce5abddb317db7dfffa95856604bfac2a17f", null ],
      [ "kStatus_LPUART_RxBusy", "group__lpuart__driver.html#gga91d929761e975dda91c0cc811d253ce5a98dfeab0a23c81f6b7f2c18acff8cc30", null ],
      [ "kStatus_LPUART_TxIdle", "group__lpuart__driver.html#gga91d929761e975dda91c0cc811d253ce5a079c6d0957d121ce7cd95cf97487dfbb", null ],
      [ "kStatus_LPUART_RxIdle", "group__lpuart__driver.html#gga91d929761e975dda91c0cc811d253ce5a02b8fe84bbfd8c52856b5d48865f7088", null ],
      [ "kStatus_LPUART_TxWatermarkTooLarge", "group__lpuart__driver.html#gga91d929761e975dda91c0cc811d253ce5abbee969cb0f7b4c49b44b10c4eb583cc", null ],
      [ "kStatus_LPUART_RxWatermarkTooLarge", "group__lpuart__driver.html#gga91d929761e975dda91c0cc811d253ce5aca81b4d6e2e792c6d841b95ec25daad9", null ],
      [ "kStatus_LPUART_FlagCannotClearManually", "group__lpuart__driver.html#gga91d929761e975dda91c0cc811d253ce5a07d5a5092ded158fe53ea4df940f8bdb", null ],
      [ "kStatus_LPUART_Error", "group__lpuart__driver.html#gga91d929761e975dda91c0cc811d253ce5af069a1997c3d25588d834ccb33dd35f4", null ],
      [ "kStatus_LPUART_RxRingBufferOverrun", "group__lpuart__driver.html#gga91d929761e975dda91c0cc811d253ce5a45a7ed684f4c31aec8999b3da68d3b5e", null ],
      [ "kStatus_LPUART_RxHardwareOverrun", "group__lpuart__driver.html#gga91d929761e975dda91c0cc811d253ce5af8ada3eacff87751ec865b4fc584bac1", null ],
      [ "kStatus_LPUART_NoiseError", "group__lpuart__driver.html#gga91d929761e975dda91c0cc811d253ce5a6ccebae2eb859fde044f15f1dd18cba1", null ],
      [ "kStatus_LPUART_FramingError", "group__lpuart__driver.html#gga91d929761e975dda91c0cc811d253ce5a1528dc2aacb3a792a1ecbfbbc7941bc9", null ],
      [ "kStatus_LPUART_ParityError", "group__lpuart__driver.html#gga91d929761e975dda91c0cc811d253ce5ad42bcd27cb6e04725768c6b98d4cbe0d", null ],
      [ "kStatus_LPUART_BaudrateNotSupport", "group__lpuart__driver.html#gga91d929761e975dda91c0cc811d253ce5aaa020183f56b7d7f63e22fd5fd8d3df2", null ]
    ] ],
    [ "lpuart_parity_mode_t", "group__lpuart__driver.html#ga6fcd73abf900b8a17dab1f2b1f3f53c7", [
      [ "kLPUART_ParityDisabled", "group__lpuart__driver.html#gga6fcd73abf900b8a17dab1f2b1f3f53c7a0e5bdb8fc4e1010930d05e1490a75b1a", null ],
      [ "kLPUART_ParityEven", "group__lpuart__driver.html#gga6fcd73abf900b8a17dab1f2b1f3f53c7ac92adb3fdf11240ca5dee6c6543b6ff7", null ],
      [ "kLPUART_ParityOdd", "group__lpuart__driver.html#gga6fcd73abf900b8a17dab1f2b1f3f53c7adab63856e98e0715f4ab289ac0da3575", null ]
    ] ],
    [ "lpuart_data_bits_t", "group__lpuart__driver.html#ga9c4d95a2016ff0cb826d50035a1ba216", [
      [ "kLPUART_EightDataBits", "group__lpuart__driver.html#gga9c4d95a2016ff0cb826d50035a1ba216aaa10fe51f0b83d5c0a2e060dab983899", null ]
    ] ],
    [ "lpuart_stop_bit_count_t", "group__lpuart__driver.html#ga7540d87bc3fa8a605d8da53fe08219f0", [
      [ "kLPUART_OneStopBit", "group__lpuart__driver.html#gga7540d87bc3fa8a605d8da53fe08219f0a4304f0ee73a5fabfadfc05ca1fbd3901", null ],
      [ "kLPUART_TwoStopBit", "group__lpuart__driver.html#gga7540d87bc3fa8a605d8da53fe08219f0a6e049e0faaf89ed2dfe493ad8f6c93d4", null ]
    ] ],
    [ "_lpuart_interrupt_enable", "group__lpuart__driver.html#ga199a157d391291a9d003bf23954f9603", [
      [ "kLPUART_RxActiveEdgeInterruptEnable", "group__lpuart__driver.html#gga199a157d391291a9d003bf23954f9603a43750aac20f72535350c1ab4a1862a3a", null ],
      [ "kLPUART_TxDataRegEmptyInterruptEnable", "group__lpuart__driver.html#gga199a157d391291a9d003bf23954f9603a0e8bf389f65e5f86a3063dc55b8aae1c", null ],
      [ "kLPUART_TransmissionCompleteInterruptEnable", "group__lpuart__driver.html#gga199a157d391291a9d003bf23954f9603a0b7f783393a61cce88eab28307e9fe14", null ],
      [ "kLPUART_RxDataRegFullInterruptEnable", "group__lpuart__driver.html#gga199a157d391291a9d003bf23954f9603a5b4a0893c98bf79d0dad88aeb7714d22", null ],
      [ "kLPUART_IdleLineInterruptEnable", "group__lpuart__driver.html#gga199a157d391291a9d003bf23954f9603af3f83c56032b14e50a682857dd7c152d", null ],
      [ "kLPUART_RxOverrunInterruptEnable", "group__lpuart__driver.html#gga199a157d391291a9d003bf23954f9603a8a29c714eaa6acab06a87314f4f29636", null ],
      [ "kLPUART_NoiseErrorInterruptEnable", "group__lpuart__driver.html#gga199a157d391291a9d003bf23954f9603ad1a35bc7e89f170a5b82cb4801b73b5e", null ],
      [ "kLPUART_FramingErrorInterruptEnable", "group__lpuart__driver.html#gga199a157d391291a9d003bf23954f9603a1b43ca06e0af6b0fcf4b55612363a64d", null ],
      [ "kLPUART_ParityErrorInterruptEnable", "group__lpuart__driver.html#gga199a157d391291a9d003bf23954f9603a13c12b816605deaabcbc3e6a6db82466", null ]
    ] ],
    [ "_lpuart_flags", "group__lpuart__driver.html#ga24bb7ca3f894fe5ff55b9f38bec16c89", [
      [ "kLPUART_TxDataRegEmptyFlag", "group__lpuart__driver.html#gga24bb7ca3f894fe5ff55b9f38bec16c89aa765f8ddac00348328f64ad4222d057a", null ],
      [ "kLPUART_TransmissionCompleteFlag", "group__lpuart__driver.html#gga24bb7ca3f894fe5ff55b9f38bec16c89ae0539c4cf39d4d1e7839a4896fad2a85", null ],
      [ "kLPUART_RxDataRegFullFlag", "group__lpuart__driver.html#gga24bb7ca3f894fe5ff55b9f38bec16c89a9bc679bcc20b81768f84b04e499c5a2f", null ],
      [ "kLPUART_IdleLineFlag", "group__lpuart__driver.html#gga24bb7ca3f894fe5ff55b9f38bec16c89acf1a40aca38878bc395f9f0b10d8225e", null ],
      [ "kLPUART_RxOverrunFlag", "group__lpuart__driver.html#gga24bb7ca3f894fe5ff55b9f38bec16c89a994f5f484509133c632215a8b701f8d1", null ],
      [ "kLPUART_NoiseErrorFlag", "group__lpuart__driver.html#gga24bb7ca3f894fe5ff55b9f38bec16c89a2198232cb9d239f937ab221bbf03259f", null ],
      [ "kLPUART_FramingErrorFlag", "group__lpuart__driver.html#gga24bb7ca3f894fe5ff55b9f38bec16c89ad1d40f89cb6aab9c12113586fda7f510", null ],
      [ "kLPUART_ParityErrorFlag", "group__lpuart__driver.html#gga24bb7ca3f894fe5ff55b9f38bec16c89a2f8d4c03df61bc4678b2ccf1c69eedeb", null ],
      [ "kLPUART_RxActiveEdgeFlag", "group__lpuart__driver.html#gga24bb7ca3f894fe5ff55b9f38bec16c89ae89f181d8dbbf5254d6fe843e4cac6d6", null ],
      [ "kLPUART_RxActiveFlag", "group__lpuart__driver.html#gga24bb7ca3f894fe5ff55b9f38bec16c89a1f14711a74db1cc141b5c2627ee4c799", null ]
    ] ],
    [ "LPUART_Init", "group__lpuart__driver.html#ga26ce3e5d63d8dd5d317a83f22af9682c", null ],
    [ "LPUART_Deinit", "group__lpuart__driver.html#ga303ff3b77767bafa449d96f1e8c921e0", null ],
    [ "LPUART_GetDefaultConfig", "group__lpuart__driver.html#ga95dfe3c3886692f1fe32023d59708440", null ],
    [ "LPUART_SetBaudRate", "group__lpuart__driver.html#gaf03d9292f8b4cb6e9748cb0bc1db7577", null ],
    [ "LPUART_GetStatusFlags", "group__lpuart__driver.html#gab83db963591b116f90daf3e24118cb09", null ],
    [ "LPUART_ClearStatusFlags", "group__lpuart__driver.html#gac6bcdb29d31a2addb8d80d98855ba147", null ],
    [ "LPUART_EnableInterrupts", "group__lpuart__driver.html#gab5998ebb0dc457f0864c84c16f83d745", null ],
    [ "LPUART_DisableInterrupts", "group__lpuart__driver.html#ga2540406e4a338199acddaa828829cad8", null ],
    [ "LPUART_GetEnabledInterrupts", "group__lpuart__driver.html#ga172b524bae8bacc6dc95d9daf82c97b5", null ],
    [ "LPUART_EnableTx", "group__lpuart__driver.html#ga49961d4e9043ff9bd10e477add9258d7", null ],
    [ "LPUART_EnableRx", "group__lpuart__driver.html#gaadb8a7199526555e50e65e017da4eae2", null ],
    [ "LPUART_WriteByte", "group__lpuart__driver.html#ga9a9389e1c7bcce6c23e00eb77006a717", null ],
    [ "LPUART_ReadByte", "group__lpuart__driver.html#ga02013105f0aacc7062eaae59d7401d29", null ],
    [ "LPUART_WriteBlocking", "group__lpuart__driver.html#gac5377aeebf4327f4ef9de295d8695cd9", null ],
    [ "LPUART_ReadBlocking", "group__lpuart__driver.html#gae76aba14dbfe94124082785e2c0ecd9d", null ],
    [ "LPUART_TransferCreateHandle", "group__lpuart__driver.html#ga10eae12610523a624c1016882f8dba5f", null ],
    [ "LPUART_TransferSendNonBlocking", "group__lpuart__driver.html#gafba986c473e5935131e63cb882cf26ff", null ],
    [ "LPUART_TransferStartRingBuffer", "group__lpuart__driver.html#ga5257b77491042af5913a8d91c66318e1", null ],
    [ "LPUART_TransferStopRingBuffer", "group__lpuart__driver.html#gad6a230a664808592aab153ea20e3d60b", null ],
    [ "LPUART_TransferAbortSend", "group__lpuart__driver.html#ga6dfe9efe9656e126c70ee79fa03f2be3", null ],
    [ "LPUART_TransferGetSendCount", "group__lpuart__driver.html#gad80d2469dce0de9361e731e4cefde9fd", null ],
    [ "LPUART_TransferReceiveNonBlocking", "group__lpuart__driver.html#ga8bdb584704ce7955004751e32627f918", null ],
    [ "LPUART_TransferAbortReceive", "group__lpuart__driver.html#ga4950fa0f3835992251c109f655a3ca7d", null ],
    [ "LPUART_TransferGetReceiveCount", "group__lpuart__driver.html#gabfcdc658c463e9e7523cc60c5e8f9672", null ],
    [ "LPUART_TransferHandleIRQ", "group__lpuart__driver.html#gac81ce3c490d7185ab7e2d97963ae077e", null ],
    [ "LPUART_TransferHandleErrorIRQ", "group__lpuart__driver.html#ga21345340b5d8f90df6bb64acab0f870f", null ]
];