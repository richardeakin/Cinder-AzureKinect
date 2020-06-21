#include "imguix/ImGuiFilePicker.h"
#include "imgui/imgui.h"

#include <algorithm>
#include <locale>
#include <sstream>

using namespace cinder;
using namespace std;

namespace imx {

FilePicker::FilePicker( const cinder::fs::path &initialPath )
{
	if( initialPath.empty() ) {
		changeWorkingDirectory( fs::current_path() );
	}
	else {
		changeWorkingDirectory( initialPath );
	}
}

bool FilePicker::show()
{
	if( ImGui::InputText( "Current Dir", &mCurrentDirectoryBuffer[0], mCurrentDirectoryBuffer.size() ) ) {
		changeWorkingDirectory( fs::path( &mCurrentDirectoryBuffer[0] ) );
		return false;
	}

	if( ImGui::Button( "Parent Directory" ) ) {
		fs::path currentDirectory( &mCurrentDirectoryBuffer[0] );
		changeWorkingDirectory( currentDirectory.parent_path() );
		return false;
	}

	if( ImGui::Checkbox( "Show only MKV files", &mFilterExtensions ) ) {
		fs::path currentDirectory( &mCurrentDirectoryBuffer[0] );
		changeWorkingDirectory( currentDirectory );
	}

	for( const std::string& currentSubdirectory : mCurrentDirectorySubdirectories ) {
		std::stringstream labelBuilder;
		labelBuilder << "> " << currentSubdirectory;
		if( ImGui::SmallButton( labelBuilder.str().c_str() ) ) {
			fs::path newWorkingDirectory( &mCurrentDirectoryBuffer[0] );
			newWorkingDirectory.append( currentSubdirectory.c_str() );

			changeWorkingDirectory( newWorkingDirectory );
			return false;
		}
	}

	bool wasSelected = false;
	for( const std::string& currentFile : mCurrentDirectoryFiles ) {
		std::stringstream labelBuilder;
		labelBuilder << "  " << currentFile;
		if( ImGui::Selectable( labelBuilder.str().c_str() ) ) {
			mSelectedPath = fs::path( &mCurrentDirectoryBuffer[0] ).append( currentFile.c_str() );
			wasSelected = true;
			break;
		}
	}

	return wasSelected;
}

void FilePicker::changeWorkingDirectory( fs::path newDirectory )
{
	if( ! fs::is_directory( newDirectory ) ) {
		return;
	}
	std::string newDirectoryStr = newDirectory.string();
	if( newDirectoryStr.size() + 1 > mCurrentDirectoryBuffer.size() ) {
		return;
	}

	const char* newDirectoryCStr = newDirectoryStr.c_str();
	std::copy( newDirectoryCStr, newDirectoryCStr + newDirectoryStr.size() + 1, mCurrentDirectoryBuffer.begin() );

	mCurrentDirectoryFiles.clear();
	mCurrentDirectorySubdirectories.clear();

	for( const auto &entry : fs::directory_iterator( newDirectory ) ) {
		const auto &p = entry.path();
		if( entry.is_directory() ) {
			mCurrentDirectorySubdirectories.emplace_back( p.filename().string() );
		}
		else {
			std::string extension = p.extension().string();
			std::transform( extension.begin(), extension.end(), extension.begin(), []( const char& c ) {
				return std::tolower( c, std::locale() );
			} );
			if( ! mFilterExtensions || extension == ".mkv" ) {
				mCurrentDirectoryFiles.emplace_back( p.filename().string() );
			}
		}
	}

	// Directories are not guaranteed to be returned in sorted order on all platforms
	//
	std::sort( mCurrentDirectoryFiles.begin(), mCurrentDirectoryFiles.end() );
	std::sort( mCurrentDirectorySubdirectories.begin(), mCurrentDirectorySubdirectories.end() );
}

} // namespace imx
