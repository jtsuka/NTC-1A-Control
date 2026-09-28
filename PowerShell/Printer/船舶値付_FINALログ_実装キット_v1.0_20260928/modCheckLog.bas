Attribute VB_Name = "modCheckLog"
Option Explicit

' ============================================================
' 船舶値付 FINAL CheckLog - 検証コピー専用
' 原典VBAの帳票ロジックは変更せず、印刷直前の最終セル状態を観測する。
'
' 重要:
'   - 原典xlsmには追加しない。必ず検証コピーにだけImportする。
'   - Touchは「実際にセルへ書いた直後」にだけ呼ぶ。
'   - TouchではCSVへ書かない。PrintOut直前のFlushで最終値を読む。
' ============================================================

Private mEnabled As Boolean
Private mOpened As Boolean
Private mFileNo As Integer
Private mLogPath As String
Private mCompletePath As String
Private mRunId As String
Private mEntries As Object          ' Scripting.Dictionary (late binding)
Private mFileDate As String

Public Sub CheckLog_Begin(ByVal outputFolder As String, Optional ByVal fileDateText As String = "")
    On Error GoTo EH

    mEnabled = False
    mOpened = False
    mFileDate = fileDateText

    CheckLog_EnsureFolder outputFolder

    mRunId = Format$(Now, "yyyymmdd_HHMMSS")
    mLogPath = outputFolder & "\VBA_FINAL_CheckLog_" & mRunId & ".csv"
    mCompletePath = outputFolder & "\VBA_FINAL_CheckLog_" & mRunId & ".complete"

    Set mEntries = CreateObject("Scripting.Dictionary")
    mEntries.CompareMode = 1 ' TextCompare

    mFileNo = FreeFile
    Open mLogPath For Output As #mFileNo
    mOpened = True

    Print #mFileNo, "RunId,BT_NO,OrderNo,PrintSequence,DetailNo,LineNo,Area,Position,ActualCell,SourceId,Value2,Text,TextHashFlag,X,Y,Width,Height,MergeAddress,SheetName,FILE_DATE,Timestamp"

    mEnabled = True
    Exit Sub
EH:
    mEnabled = False
    On Error Resume Next
    If mOpened Then Close #mFileNo
    mOpened = False
    On Error GoTo 0
End Sub

Public Sub CheckLog_ResetOrder()
    On Error GoTo EH
    If Not mEnabled Then Exit Sub
    Set mEntries = CreateObject("Scripting.Dictionary")
    mEntries.CompareMode = 1
    Exit Sub
EH:
    ' ログ障害を原典処理へ伝播させない
End Sub

' 実際に書込みを行ったセルだけ登録する。
' Position  = setting表M列の基準アドレス (例 BJ12)
' ActualCell= RangeCalculation後の実セル (例 BJ18)
Public Sub CheckLog_Touch( _
    ByVal areaName As String, _
    ByVal position As String, _
    ByVal sourceId As String, _
    ByVal target As Range, _
    ByVal detailNo As Long, _
    ByVal lineNo As String)

    On Error GoTo EH
    If Not mEnabled Then Exit Sub
    If target Is Nothing Then Exit Sub

    Dim key As String
    Dim actualCell As String
    Dim sheetName As String
    Dim v As Variant

    actualCell = target.Address(False, False)
    sheetName = target.Worksheet.Name
    key = CStr(detailNo) & "|" & areaName & "|" & position

    ' 後勝ちを許容する。BJ12のid=211がid=12を上書きした場合も
    ' FINAL Flushでは同じPositionを1行だけ読み返す。
    v = Array(areaName, position, actualCell, sourceId, CStr(detailNo), CStr(lineNo), sheetName)
    mEntries(key) = v
    Exit Sub
EH:
    ' fail-soft
End Sub

Public Sub CheckLog_FlushOrder( _
    ByVal ws As Worksheet, _
    ByVal btNo As Long, _
    ByVal orderNo As String, _
    ByVal printSequence As Long, _
    Optional ByVal fileDateText As String = "")

    On Error GoTo EH
    If Not mEnabled Then Exit Sub
    If mEntries Is Nothing Then Exit Sub

    If Len(fileDateText) > 0 Then mFileDate = fileDateText

    Dim keys As Variant
    Dim i As Long
    Dim a As Variant
    Dim target As Range
    Dim r As Range
    Dim value2Text As String
    Dim displayText As String
    Dim hashFlag As String
    Dim mergeAddress As String
    Dim ts As String

    keys = mEntries.Keys
    For i = LBound(keys) To UBound(keys)
        a = mEntries(keys(i))

        Set target = ws.Range(CStr(a(2)))
        If target.MergeCells Then
            Set r = target.MergeArea
        Else
            Set r = target
        End If

        value2Text = CheckLog_Value2ToText(r.Cells(1, 1).Value2)
        displayText = CStr(r.Cells(1, 1).Text)

        If Left$(displayText, 1) = "#" Then
            hashFlag = "1"
        Else
            hashFlag = "0"
        End If

        If r.MergeCells Then
            mergeAddress = r.Address(False, False)
        Else
            mergeAddress = ""
        End If

        ts = Format$(Now, "yyyy/mm/dd HH:nn:ss")

        Print #mFileNo, _
            CheckLog_Csv(mRunId) & "," & _
            CheckLog_Csv(CStr(btNo)) & "," & _
            CheckLog_Csv(orderNo) & "," & _
            CheckLog_Csv(CStr(printSequence)) & "," & _
            CheckLog_Csv(CStr(a(4))) & "," & _
            CheckLog_Csv(CStr(a(5))) & "," & _
            CheckLog_Csv(CStr(a(0))) & "," & _
            CheckLog_Csv(CStr(a(1))) & "," & _
            CheckLog_Csv(CStr(a(2))) & "," & _
            CheckLog_Csv(CStr(a(3))) & "," & _
            CheckLog_Csv(value2Text) & "," & _
            CheckLog_Csv(displayText) & "," & _
            CheckLog_Csv(hashFlag) & "," & _
            CheckLog_Csv(CStr(r.Left)) & "," & _
            CheckLog_Csv(CStr(r.Top)) & "," & _
            CheckLog_Csv(CStr(r.Width)) & "," & _
            CheckLog_Csv(CStr(r.Height)) & "," & _
            CheckLog_Csv(mergeAddress) & "," & _
            CheckLog_Csv(CStr(a(6))) & "," & _
            CheckLog_Csv(mFileDate) & "," & _
            CheckLog_Csv(ts)
    Next i

    Exit Sub
EH:
    ' fail-soft。completeが作られない異常Runは比較対象外。
End Sub

Public Sub CheckLog_End()
    On Error GoTo EH
    If Not mEnabled Then Exit Sub

    If mOpened Then
        Close #mFileNo
        mOpened = False
    End If

    Dim f As Integer
    f = FreeFile
    Open mCompletePath For Output As #f
    Print #f, "COMPLETE," & mRunId & "," & Format$(Now, "yyyy/mm/dd HH:nn:ss")
    Close #f

    mEnabled = False
    Exit Sub
EH:
    mEnabled = False
    On Error Resume Next
    If mOpened Then Close #mFileNo
    mOpened = False
    On Error GoTo 0
End Sub

Public Function CheckLog_IsEnabled() As Boolean
    CheckLog_IsEnabled = mEnabled
End Function

Private Function CheckLog_Csv(ByVal s As String) As String
    CheckLog_Csv = """" & Replace(s, """", """""") & """"
End Function

Private Function CheckLog_Value2ToText(ByVal v As Variant) As String
    On Error GoTo EH
    If IsError(v) Then
        CheckLog_Value2ToText = CStr(v)
    ElseIf IsEmpty(v) Then
        CheckLog_Value2ToText = ""
    Else
        CheckLog_Value2ToText = CStr(v)
    End If
    Exit Function
EH:
    CheckLog_Value2ToText = ""
End Function

Private Sub CheckLog_EnsureFolder(ByVal folderPath As String)
    On Error GoTo EH
    If Len(Dir$(folderPath, vbDirectory)) = 0 Then MkDir folderPath
    Exit Sub
EH:
    Err.Raise Err.Number, "CheckLog_EnsureFolder", Err.Description
End Sub
