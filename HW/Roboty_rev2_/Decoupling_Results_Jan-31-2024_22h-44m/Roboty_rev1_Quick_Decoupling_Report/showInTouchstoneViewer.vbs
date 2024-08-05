Dim ResFile
If IsObject(WScript) Then
	ResFile = WScript.Arguments(0)	
Else
	' Linux
	ResFile = ScriptHelper.Arguments.item(3)
	ResFile = Replace(ResFile, Chr(34), "")
End If

Const tvPlotTypeMag = 4
Const tvDirHorz = 1
Const tvDirVert = 2

Set TVApp = GetApp("TouchstoneViewer.TVApplication", "105")

Dim fso: Set fso = CreateObject("Scripting.FileSystemObject")
Dim CurrentDir : CurrentDir = fso.GetAbsolutePathName("..")
Dim FilePath : FilePath = fso.BuildPath(CurrentDir, ResFile)

TVApp.BringToTop
TVApp.GUI.PlotType = tvPlotTypeMag

' Disable if anything else open
nCount = TVApp.GetModelCount()
For i = 1 To nCount
	Set OpenModel = TVApp.GetModel(i)
	TVApp.GUI.Enable OpenModel, False
Next

Set Model = TVApp.OpenFile(FilePath)
If Model.Dimension < 20 Then
    TVApp.GUI.CheckAll True
End If

If TVApp.Viewer.CanLogScale(tvDirHorz) Then
    TVApp.Viewer.SetLogScale tvDirHorz, True
End If
If TVApp.Viewer.CanLogScale(tvDirVert) Then
    TVApp.Viewer.SetLogScale tvDirVert, True
End If

CreateObject("WScript.Shell").AppActivate("HyperLynx Touchstone and Fitted-Poles Viewer - VX.2.10")

Function GetApp(AppName, Version)
	On error resume next
		Set GetApp = GetObject("", AppName + "." + Version)
		If err.number <> 0 Then
			Set GetApp = GetObject("", AppName)
		End If
	On error goto 0	
End Function
